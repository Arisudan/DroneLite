import time
import unittest

from nimbus_core import topics as T
from nimbus_core.core import NimbusCore
from nimbus_core.throttle_ramp import RampConfig


def run_ramp(cfg, sim_hover=0.42, tilt_at=None, timeout=90):
    from nimbus_pc.hover.sim import SimParams
    core = NimbusCore(); core.connect_sim(SimParams(hover_thr=sim_hover)); core.set_source("external"); core.set_allow_arm(True)
    core.ramp.cfg = cfg
    try:
        t0 = time.monotonic(); started = False
        while time.monotonic() - t0 < timeout:
            core.tick(); time.sleep(0.02)
            if not started and time.monotonic() - t0 > 2.5:
                started = core.ramp.start(); assert started
            if tilt_at and core.ramp.phase == "ramping" and core.ramp.thr >= tilt_at:
                core.safety.feed_attitude(time.monotonic(), 90.0, 0.0)
                core._feed_safety(("attitude", {"pitch_rad": 1.6, "roll_rad": 0.0, "yaw_rad": 0.0}))
            if started and core.ramp.phase == "done":
                break
        res = core.ramp.result
        return res, core
    finally:
        core.disconnect()


class TestRamp(unittest.TestCase):
    def test_finds_the_lift_throttle_just_above_the_sim_hover_throttle(self):
        res, _ = run_ramp(RampConfig(rate_per_s=0.04, cap_thr=0.6))
        self.assertEqual(res["reason"], "lift")
        self.assertGreater(res["lift_throttle"], 0.40); self.assertLess(res["lift_throttle"], 0.55)
        self.assertLess(res["hover_throttle_est"], res["lift_throttle"])

    def test_cap_below_the_lift_throttle_stops_with_limit_and_learns_nothing(self):
        res, _ = run_ramp(RampConfig(rate_per_s=0.06, cap_thr=0.30))
        self.assertEqual(res["reason"], "limit"); self.assertIsNone(res["lift_throttle"])

    def test_holds_at_the_cap_before_giving_up(self):
        res, _ = run_ramp(RampConfig(start_thr=0.30, rate_per_s=0.5, cap_thr=0.30, hold_at_cap_s=2.0))
        self.assertEqual(res["reason"], "limit"); self.assertGreaterEqual(res["seconds"], 2.0 + 1.5)       # arming + the 2 s hold

    def test_extreme_tilt_stops_the_ramp(self):
        res, core = run_ramp(RampConfig(rate_per_s=0.05, cap_thr=0.6), tilt_at=0.2)
        self.assertTrue(res["reason"].startswith("safety_disarm") or res["reason"] == "tilt", res["reason"])
        self.assertFalse(core.last_controls.arm)

    def test_refuses_to_start_without_arming_allowed_or_external_source(self):
        core = NimbusCore(); core.connect_sim()
        try:
            self.assertFalse(core.ramp.start())
        finally:
            core.disconnect()


if __name__ == "__main__":
    unittest.main()


class TestRadioProbe(unittest.TestCase):
    def test_reads_motors_and_blockers_over_the_radio_without_blocking_the_controls(self):
        from nimbus_core.radio_fc import RadioFcProbe
        from nimbus_pc.emulator import DongleEmulator
        emu = DongleEmulator(); emu.motor_values = [1300, 1310, 1320, 1330]; emu.arming_flags = (1 << 12) | (1 << 2); emu.start()
        core = NimbusCore()
        try:
            core.connect(emu.path)
            probe = RadioFcProbe(core.sess, period_s=0.1); probe.start()
            gaps = []; last = time.monotonic(); t0 = last
            while time.monotonic() - t0 < 3.0:                              # the control tick must keep running at full speed meanwhile
                core.tick(); time.sleep(0.02)
                now = time.monotonic(); gaps.append(now - last); last = now
            probe.stop()
            self.assertEqual(probe.motors, [1300, 1310, 1320, 1330])
            self.assertEqual(sorted(probe.blockers), ["CALIB", "RXLOSS"])
            self.assertGreater(probe.replies, 10); self.assertLess(max(gaps), 0.15)
            self.assertFalse(core.sess.sender.failsafe_active)
            with self.assertRaises(AssertionError): probe._ask(11)           # anything but 104/150 is refused
        finally:
            core.disconnect(); emu.stop()


class TestSoftStart(unittest.TestCase):
    def test_a_high_start_value_is_reached_smoothly_not_in_one_step(self):
        res, core = run_ramp(RampConfig(start_thr=0.30, rate_per_s=0.5, cap_thr=0.30, hold_at_cap_s=0.5))
        thr = [row[1] for row in core.ramp.log]
        self.assertLess(thr[0], 0.05)                                             # first ramp frame is near zero
        self.assertTrue(all(b - a <= 0.30 * 0.06 + 1e-9 for a, b in zip(thr, thr[1:])))   # never more than 30 %/s
        self.assertAlmostEqual(thr[-1], 0.30, places=3)


class TestRecorder(unittest.TestCase):
    def test_a_sim_flight_is_recorded_with_events_and_controller_columns(self):
        import csv
        core = NimbusCore(); core.connect_sim(); core.set_source("external"); core.set_allow_arm(True)
        try:
            t0 = time.monotonic(); took = False
            while time.monotonic() - t0 < 16:
                core.tick(); time.sleep(0.02)
                if not took and time.monotonic() - t0 > 4: took = core.autonomy.takeoff()
            path = core.recorder.path
            core.autonomy.land()
        finally:
            core.disconnect()
        rows = list(csv.DictReader(open(path)))
        self.assertGreater(len(rows), 200)
        states = {r["state"] for r in rows}
        self.assertTrue({"takeoff", "sign_check"} <= states)
        self.assertTrue(any(r["cmd_arm"] == "1" and float(r["z_est"] or 0) > 0.05 for r in rows))
        self.assertTrue(all(r["range_m"] != "" for r in rows[-50:]))
        ev = open(path.replace(".csv", ".events.txt")).read()
        self.assertIn("AUTONOMY takeoff requested", ev); self.assertIn("connected to the SIMULATOR", ev)


class TestRecorderFcState(unittest.TestCase):
    def test_fc_kinematic_frames_are_logged_next_to_our_estimate(self):
        import csv
        core = NimbusCore(); core.connect_sim()
        try:
            for i in range(30):
                core.bus.publish(T.TELEMETRY, ("kinematic", {"x_m": 0.1, "y_m": -0.2, "z_m": 0.5, "vx_mps": 0.0, "vy_mps": 0.01, "vz_mps": 0.0, "roll_rad": 0.0, "pitch_rad": 0.0, "yaw_rad": 0.0, "valid": True}), cache=False)
                core.tick(); time.sleep(0.02)
            path = core.recorder.path
        finally:
            core.disconnect()
        rows = list(csv.DictReader(open(path)))
        self.assertTrue(any(r["fc_z"] == "0.5" and r["fc_valid"] == "1" and r["fc_y"] == "-0.2" for r in rows))
        self.assertIn("est_x", rows[0])


class TestGyroCalibrationBeforeArming(unittest.TestCase):
    def test_live_ramp_sends_the_disarmed_gyro_gesture_first_then_arms(self):
        from nimbus_core.throttle_ramp import ThrottleRamp
        from nimbus_pc.sender import Controls

        class FakeAuto: fc_armed = False
        class FakeSafety: disarm_latched = None
        class FakeCore:
            connected = True; allow_arm = True; source = "external"; state_source = "live"
            autonomy, safety = FakeAuto(), FakeSafety()
            def __init__(self): self.bus = type("B", (), {"subscribe": lambda s, *a: None})(); self.events = []
            def event(self, t): self.events.append(t)
            def set_external_controls(self, c): pass
        core = FakeCore(); r = ThrottleRamp(core, RampConfig()); r._lq = 100
        assert r.start()
        t0 = r._t0
        first = r.tick(t0 + 0.5)
        self.assertEqual((first.pitch, first.yaw, first.throttle, first.arm), (-1.0, -1.0, 0.0, False))      # the gesture, DISARMED, throttle minimum
        self.assertEqual(r.tick(t0 + 3.0), Controls())                                                       # then stillness (sticks centred, arm low)
        self.assertTrue(r.tick(t0 + 4.2).arm)                                                                # then the arm command at zero throttle
        for t in (0.5, 3.0):
            c = r.tick(t0 + t); self.assertFalse(c.arm); self.assertEqual(c.throttle, 0.0)
