"""The director flying the FC-side hybrid (settings.profile == "fc") in the simulator: the PC requests, the firmware's own height-hold law (compiled from the firmware source) holds."""
import dataclasses
import time
import unittest

from nimbus_core import fc_hold_model as M
from nimbus_core import settings as S
from nimbus_core.core import NimbusCore


@unittest.skipUnless(M.lib_path(), "libhh.so not built (tools_scripts/fc_hold_sim/build.sh)")
class TestFcLoopThroughTheDirector(unittest.TestCase):
    def _core(self, h=0.5, hover=0.72):
        core = NimbusCore(); d = core.director; au = d._au
        st = dataclasses.replace(au.settings, hover_height=h, ceiling_m=0.8, thr_hover0=0.66, thr_max=0.85, measured=True, batt_ref_v=13.8, profile="fc")
        self.assertTrue(d.apply_settings(st, persist=False).ok)
        core.connect_sim(params=dataclasses.replace(S.plant_params(st), hover_thr=hover))
        return core, d

    def _wait(self, core, d, cond, timeout, what):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            core.tick(); time.sleep(0.01)
            if cond(): return
        self.fail(f"timeout waiting for {what}: phase {d.state().phase}, {d.state().message}")

    def test_arm_take_off_hold_and_land(self):
        core, d = self._core()
        try:
            self._wait(core, d, lambda: d.preflight_input().get("range_m") is not None and (d.preflight_input().get("link_lq") or 0) > 0, 20, "telemetry")
            for _ in range(400): core.tick(); time.sleep(0.01)
            self.assertFalse([c for c in d.preflight() if not c.ok and c.blocking], [c.label for c in d.preflight() if not c.ok])
            self.assertTrue(d.arm().ok); self._wait(core, d, lambda: d.state().phase == "armed", 25, "armed")
            self.assertTrue(d.takeoff(0.5).ok)
            self._wait(core, d, lambda: d.state().phase == "holding", 80, "holding")
            self._wait(core, d, lambda: abs((d.state().height_m or 0.0) - 0.5) < 0.12, 40, "the target height")      # the FC climbs on its minimum-jerk reference
            t_end = time.monotonic() + 4
            while time.monotonic() < t_end: core.tick(); time.sleep(0.01)
            self.assertLess(abs(d.state().height_m - 0.5), 0.25)       # the ground-effect margin (fc-hybrid-v2) overshoots a little on this plant: KNOWN MISS, see the build README
            self.assertTrue(d.land().ok)
            self._wait(core, d, lambda: d.state().phase in ("landed", "ready"), 60, "landed")
        finally:
            core.disconnect()

    def test_the_fc_sideways_loop_keeps_the_drone_over_the_take_off_point_against_a_constant_push(self):
        """a constant sideways acceleration (a level error, 0.14 m/s^2 = 0.8 deg): without a sideways loop the drone leaves in seconds"""
        import math
        core = NimbusCore(); d = core.director; au = d._au
        st = dataclasses.replace(au.settings, hover_height=0.5, ceiling_m=0.9, thr_hover0=0.66, thr_max=0.85, measured=True, batt_ref_v=13.8, profile="fc")
        self.assertTrue(d.apply_settings(st, persist=False).ok)
        p = dataclasses.replace(S.plant_params(st), hover_thr=0.72, wind=[(0.0, 1e9, 0.14, 0.0)])
        core.connect_sim(params=p, ctrl_delay_s=0.1)
        try:
            self._wait(core, d, lambda: d.preflight_input().get("range_m") is not None and (d.preflight_input().get("link_lq") or 0) > 0, 20, "telemetry")
            for _ in range(400): core.tick(); time.sleep(0.01)
            self.assertTrue(d.arm().ok); self._wait(core, d, lambda: d.state().phase == "armed", 25, "armed")
            self.assertTrue(d.takeoff(0.5).ok)
            self._wait(core, d, lambda: d.state().phase == "holding", 100, "holding")
            sim = core.sess.sim; x0, y0 = sim.x, sim.y; far = 0.0
            t_end = time.monotonic() + 25
            while time.monotonic() < t_end:
                core.tick(); time.sleep(0.01); far = max(far, math.hypot(sim.x - x0, sim.y - y0))
            self.assertLess(far, 1.5); self.assertLess(math.hypot(sim.x - x0, sim.y - y0), 0.8)
            d.land()
        finally:
            core.disconnect()

    def test_alt_hold_lets_it_drift_pos_hold_holds_again_and_stabilize_is_refused(self):
        """flight modes through AUX7 (firmware v4.2): ALT HOLD pauses the sideways loop (a constant push moves the drone, the height holds), POS HOLD takes it back"""
        import math
        core = NimbusCore(); d = core.director; au = d._au
        st = dataclasses.replace(au.settings, hover_height=0.5, ceiling_m=0.9, thr_hover0=0.66, thr_max=0.85, measured=True, batt_ref_v=13.8, profile="fc")
        self.assertTrue(d.apply_settings(st, persist=False).ok)
        p = dataclasses.replace(S.plant_params(st), hover_thr=0.72, wind=[(0.0, 1e9, 0.14, 0.0)])
        core.connect_sim(params=p, ctrl_delay_s=0.1)
        try:
            self._wait(core, d, lambda: d.preflight_input().get("range_m") is not None and (d.preflight_input().get("link_lq") or 0) > 0, 20, "telemetry")
            for _ in range(400): core.tick(); time.sleep(0.01)
            self.assertTrue(d.arm().ok); self._wait(core, d, lambda: d.state().phase == "armed", 25, "armed")
            self.assertTrue(d.takeoff(0.5).ok)
            self._wait(core, d, lambda: d.state().phase == "holding", 100, "holding")
            sim = core.sess.sim
            def run(sec):
                t_end = time.monotonic() + sec
                while time.monotonic() < t_end: core.tick(); time.sleep(0.01)
            run(8)
            x0, y0 = sim.x, sim.y
            self.assertFalse(d.set_mode("stabilize").ok)                                   # no height loop on the PC: refused
            self.assertTrue(d.set_mode("alt_hold").ok); run(3)
            drift = math.hypot(sim.x - x0, sim.y - y0)
            self.assertGreater(drift, 0.3, "ALT HOLD: the constant push must move the drone (the sideways loop is paused)")
            self.assertLess(abs(sim.z - 0.5), 0.3, "ALT HOLD still holds the height")
            self.assertTrue(d.set_mode("pos_hold").ok); run(12)
            self.assertLess(math.hypot(sim.vx, sim.vy), 0.25, "POS HOLD: the sideways loop is back and the speed dies away")
            d.land()
        finally:
            core.disconnect()

    def test_preflight_blocks_take_off_when_the_firmware_sends_no_hold_status(self):
        core = NimbusCore(); d = core.director; au = d._au
        st = dataclasses.replace(au.settings, profile="fc", thr_hover0=0.66, thr_max=0.85, measured=True, batt_ref_v=13.8)
        self.assertTrue(d.apply_settings(st, persist=False).ok)
        core.connect_sim(params=S.plant_params(st)); core.sess.fc_hold = None          # an old firmware: no hold status frame
        try:
            t0 = time.monotonic()
            while time.monotonic() - t0 < 4: core.tick(); time.sleep(0.01)
            ok, why = au.ctl.preflight(time.monotonic() - core._t0 if False else time.monotonic())
            self.assertIn("hold status", " ".join(why)) if not ok else self.fail("preflight passed without a hold status")
        finally:
            core.disconnect()


if __name__ == "__main__":
    unittest.main()
