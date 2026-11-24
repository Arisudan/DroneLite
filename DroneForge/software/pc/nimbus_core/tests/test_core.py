import math
import time
import unittest

from nimbus_core import topics as T
from nimbus_core.bus import TopicBus
from nimbus_core.core import NimbusCore
from nimbus_pc.emulator import DongleEmulator
from nimbus_pc.sender import Controls


class TestBus(unittest.TestCase):
    def test_pubsub_cache_replay_and_unsubscribe(self):
        b = TopicBus(); got = []
        unsub = b.subscribe("a", got.append)
        b.publish("a", 1); b.publish("a", 2)
        self.assertEqual(got, [1, 2]); self.assertEqual(b.last("a"), 2)
        late = []
        b.subscribe("a", late.append, replay=True)
        self.assertEqual(late, [2])
        unsub(); b.publish("a", 3)
        self.assertEqual(got, [1, 2]); self.assertEqual(late, [2, 3])

    def test_uncached_topic_has_no_last_value(self):
        b = TopicBus(); b.publish("x", 1, cache=False)
        self.assertIsNone(b.last("x"))

    def test_a_broken_subscriber_never_stops_the_publisher_or_others(self):
        b = TopicBus(); ok = []
        b.subscribe("t", lambda m: 1 / 0)
        b.subscribe("t", ok.append)
        b.publish("t", 5)
        self.assertEqual(ok, [5]); self.assertEqual(b.errors, 1)


class TestCore(unittest.TestCase):
    def setUp(self):
        self.emu = DongleEmulator().start()
        self.core = NimbusCore()
        self.seen = {"tel": [], "ev": [], "conn": [], "frames": 0}
        b = self.core.bus
        b.subscribe(T.TELEMETRY, lambda m: self.seen["tel"].append(m[0]))
        b.subscribe(T.EVENT, self.seen["ev"].append)
        b.subscribe(T.CONNECTION, self.seen["conn"].append)
        b.subscribe(T.FRAME_IN, lambda f: self.seen.__setitem__("frames", self.seen["frames"] + 1))
        self.core.connect(self.emu.path)

    def tearDown(self):
        self.core.disconnect(); self.emu.stop()

    def test_connect_publishes_connection_telemetry_frames_and_events(self):
        for _ in range(40):
            self.core.tick(); time.sleep(0.03)
        self.assertEqual(self.seen["conn"][0], {"connected": True, "port": self.emu.path, "source": "live"})
        self.assertIn("link", self.seen["tel"]); self.assertIn("optrange", self.seen["tel"])
        self.assertGreater(self.seen["frames"], 20)
        self.assertTrue(any("connected to" in e for e in self.seen["ev"]))

    def test_sweep_never_arms_safe_resets_and_topics_follow(self):
        self.core.set_source("sweep"); self.core.set_allow_arm(True)
        for _ in range(20):
            self.core.tick(); time.sleep(0.02)
        self.assertFalse(self.core.last_controls.arm)
        self.assertFalse(self.core.bus.last(T.CONTROLS).arm)
        self.assertEqual(self.core.bus.last(T.SOURCE), {"source": "sweep", "allow_arm": True})
        self.core.safe_now()
        self.assertEqual(self.core.bus.last(T.SOURCE), {"source": "idle", "allow_arm": False})
        self.assertFalse(self.core.sess.sender.allow_arm)

    def test_disconnect_publishes_and_second_core_can_open_the_port_afterwards(self):
        self.core.disconnect()
        self.assertEqual(self.core.bus.last(T.CONNECTION)["connected"], False)
        other = NimbusCore(); other.connect(self.emu.path); other.disconnect()


if __name__ == "__main__":
    unittest.main()


class TestCoreSafety(unittest.TestCase):
    def setUp(self):
        self.emu = DongleEmulator().start()
        self.core = NimbusCore(); self.core.connect(self.emu.path)
        self.core.set_source("external"); self.core.set_allow_arm(True)
        self.events = []
        self.core.bus.subscribe(T.EVENT, self.events.append)

    def tearDown(self):
        self.core.disconnect(); self.emu.stop()

    def _tick(self, pitch_deg=0.0, arm=True, n=1):
        for _ in range(n):
            self.core.set_external_controls(Controls(throttle=0.3, arm=arm))
            self.core.safety.feed_attitude(time.monotonic(), pitch_deg, 0.0)
            self.core.safety.feed_link(time.monotonic(), 100.0)
            self.core.tick(); time.sleep(0.005)

    def test_external_source_arms_when_allowed_and_level(self):
        self._tick(0.0, n=3)
        self.assertTrue(self.core.last_controls.arm)
        self.assertAlmostEqual(self.core.last_controls.throttle, 0.3)

    def test_extreme_tilt_forces_arm_low_even_though_the_source_keeps_asking(self):
        self._tick(0.0, n=2)
        self._tick(80.0)
        self.assertFalse(self.core.last_controls.arm); self.assertEqual(self.core.last_controls.throttle, 0.0)
        self.assertEqual(self.core.bus.last(T.SAFETY)["disarm_latched"], "tilt_extreme")
        self._tick(0.0, n=5)                                           # level again but the source still holds arm: stays disarmed
        self.assertFalse(self.core.last_controls.arm)
        self.assertTrue(any("SAFETY [disarm]" in e for e in self.events))
        self._tick(0.0, arm=False, n=2)                                # operator drops the arm request: latch clears
        self.assertIsNone(self.core.safety.disarm_latched)
        self._tick(0.0, arm=True, n=2)
        self.assertTrue(self.core.last_controls.arm)

    def test_tilt_does_nothing_when_arming_is_not_allowed(self):
        self.core.set_allow_arm(False)
        self._tick(80.0, n=3)
        self.assertIsNone(self.core.safety.disarm_latched)

    def test_turtle_mode_switch_suppresses_the_tilt_disarm(self):
        for _ in range(3):
            self.core.set_external_controls(Controls(throttle=0.3, arm=True, aux3=1811))
            self.core.safety.feed_attitude(time.monotonic(), 170.0, 0.0)
            self.core.tick(); time.sleep(0.005)
        self.assertIsNone(self.core.safety.disarm_latched)


class TestSimAutonomy(unittest.TestCase):
    def test_takeoff_hover_and_land_on_the_sim_source(self):
        import dataclasses                                   # tests run on a temporary settings file = the UNMEASURED defaults (hover 0.42): not a flight configuration; use the measured profile, set BEFORE connect_sim so the simulated plant is built from it
        core = NimbusCore(); _au = core.director._au
        core.director.apply_settings(dataclasses.replace(_au.settings, hover_height=0.5, thr_hover0=0.63, batt_ref_v=13.8, measured=True), persist=False)
        core.connect_sim(); core.set_source("external"); core.set_allow_arm(True)
        evlog = []; core.bus.subscribe(T.EVENT, evlog.append)
        try:
            t0 = time.monotonic(); phase_seen = set(); state_seen = set(); took = False
            while time.monotonic() - t0 < 45:
                core.tick(); time.sleep(0.02)
                st = core.autonomy.status(); phase_seen.add(st["phase"]); state_seen.add(st["state"])
                if not took and time.monotonic() - t0 > 4.0:
                    self.assertTrue(core.autonomy.preflight()[0], core.autonomy.preflight()[1])
                    self.assertTrue(core.autonomy.takeoff()); took = True
                if st["state"] == "hover" and time.monotonic() - t0 > 20:
                    core.autonomy.land()
                if took and st["phase"] == "idle" and "hover" in state_seen:
                    break
            self.assertTrue({"arming", "flying"} <= phase_seen); self.assertIn("hover", state_seen)
            self.assertEqual(core.autonomy.status()["phase"], "idle")
            _sim = core.sess.sim
            self.assertFalse(_sim.crashed, f"simulated crash: max impact {_sim.max_impact:.2f} m/s, tilt x/y {math.degrees(_sim.tilt_x):.0f}/{math.degrees(_sim.tilt_y):.0f} deg, z {_sim.z:.2f}, impact at sim t {_sim.impact_t} thr_eff {_sim.impact_thr}, sim t now {_sim.t:.1f}, trace (t,z,vz,cmd_thr,thr_eff) {(_sim.impact_trace or [])[::2]}, states {sorted(state_seen)}, last events {[e[-230:] for e in evlog[-5:]]}")
            self.assertLess(core.sess.sim.z, 0.10)      # the throttle cuts at ~5 cm (the sensor over-reads); the last cm are a drop
            self.assertFalse(core.sess.fc_armed)
        finally:
            core.disconnect()

    def test_takeoff_is_refused_without_arming_allowed_or_the_right_source(self):
        core = NimbusCore(); core.connect_sim()
        try:
            time.sleep(3.0)
            self.assertFalse(core.autonomy.takeoff())                      # source idle and arming not allowed
            self.assertIn("takeoff refused", core.autonomy.message); self.assertIn("not external", core.autonomy.message)
            self.assertEqual(core.autonomy.phase, "idle")
        finally:
            core.disconnect()

    def test_tilt_safety_disarm_stops_a_running_sim_flight(self):
        core = NimbusCore(); core.connect_sim(); core.set_source("external"); core.set_allow_arm(True)
        try:
            t0 = time.monotonic(); took = False
            while time.monotonic() - t0 < 20:
                core.tick(); time.sleep(0.02)
                if not took and time.monotonic() - t0 > 4.0:
                    took = core.autonomy.takeoff()
                if took and core.autonomy.status()["state"] in ("takeoff", "sign_check") and core.sess.fc_armed:
                    core.safety.feed_attitude(time.monotonic(), 85.0, 0.0)   # the drone "flips": the safety layer must disarm
                    for _ in range(3): core.tick(); time.sleep(0.02)
                    break
            self.assertTrue(took)
            self.assertEqual(core.safety.disarm_latched, "tilt_extreme")
            self.assertFalse(core.last_controls.arm)
        finally:
            core.disconnect()


class FakeFc:
    """Stands in for the FC's MSP reader: arms when it sees arm HIGH at low throttle, motors follow the (scaled) throttle."""
    def __init__(self, core):
        self.core = core

    def armed(self):
        s = self.core.last_sent
        return bool(s.arm and self.core.allow_arm)

    def motors(self):
        s = self.core.last_sent
        if not (s.arm and self.core.allow_arm):
            return [1000] * 4
        rc = 1000 + 1000 * s.throttle                                  # CRSF throttle -> RC microseconds
        m = int(1000 + 1000 * (0.055 + 0.945 * (rc - 1050) / 950)) if rc > 1050 else 1056     # Betaflight: idle + (1-idle)*(rc-min_check)/(2000-min_check)
        return [m] * 4

    def close(self):
        pass


class TestBenchMode(unittest.TestCase):
    def test_bench_flight_uses_real_arming_scaled_throttle_and_simulated_sensors(self):
        emu = DongleEmulator(); emu.battery_dv = 160; emu.start(); core = NimbusCore()
        try:
            core.connect(emu.path, fc=FakeFc(core))
            self.assertEqual(core.state_source, "bench")
            core.set_source("external"); core.set_allow_arm(True)
            t0 = time.monotonic(); took = False; seen = set(); max_sent = 0.0
            while time.monotonic() - t0 < 50:
                core.tick(); time.sleep(0.02)
                st = core.autonomy.status(); seen.add(st["state"]); max_sent = max(max_sent, core.last_sent.throttle)
                if not took and time.monotonic() - t0 > 5:
                    ok, why = core.autonomy.preflight(); self.assertTrue(ok, why); took = core.autonomy.takeoff()
                if st["state"] == "hover" and time.monotonic() - t0 > 25:
                    core.autonomy.land()
                if took and st["phase"] == "idle" and "hover" in seen:
                    break
            self.assertIn("hover", seen)
            self.assertLessEqual(max_sent, 0.4 * 0.7 + 1e-6)             # sent throttle never exceeds the scale x thr_max
            self.assertEqual(core.autonomy.status()["phase"], "idle")
            self.assertFalse(core.bench.sim.crashed)
            self.assertLess(core.bench.sim.z, 0.12)                       # throttle cuts at ~5-8 cm (the sensor over-reads there), the last cm is a drop
        finally:
            core.disconnect(); emu.stop()
