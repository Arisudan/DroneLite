"""The PC side of the hybrid: the FC-side hold model (the firmware's own control law compiled from the firmware source), the request hook in the core, and tools_scripts/fc_hold_run.py in the simulator."""
import os
import sys
import unittest

from nimbus_core import fc_hold_model as M

HAVE = M.lib_path() is not None
SKIP = "libhh.so not built (tools_scripts/fc_hold_sim/build.sh)"


@unittest.skipUnless(HAVE, SKIP)
class TestFcHoldModel(unittest.TestCase):
    def test_idle_then_take_off_request_engages_and_reports_status_bits(self):
        from nimbus_pc.mapper import height_to_aux5, throttle01_to_crsf
        m = M.FcHoldModel(); a5, a6 = height_to_aux5(0.5), throttle01_to_crsf(0.66)
        for _ in range(40): self.assertIsNone(m.step(0.02, 0.0, 0.0, 13.8, True, 172, 172, 0.0))      # no request: the stick rules
        self.assertEqual(m.hold_bits(), 0)
        f = None
        for _ in range(10): f = m.step(0.02, 0.0, 0.0, 13.8, True, a5, a6, 0.0)
        self.assertIsNotNone(f); self.assertEqual(m.hold_bits() & 0x3C, 0x04 | 0x10)                    # active, take-off mode
        self.assertLess(f, 0.5)                                                                          # the creep starts low (a ramp, never a step)

    def test_disarm_and_missing_hover_reference_keep_it_off(self):
        from nimbus_pc.mapper import height_to_aux5
        m = M.FcHoldModel()
        for _ in range(40): m.step(0.02, 0.0, 0.0, 13.8, True, 172, 172, 0.0)
        for _ in range(10): self.assertIsNone(m.step(0.02, 0.0, 0.0, 13.8, True, height_to_aux5(0.5), 172, 0.0))      # no hover reference: no FC take-off
        self.assertIsNone(m.step(0.02, 0.0, 0.0, 13.8, False, height_to_aux5(0.5), 1000, 0.0))                           # disarmed


@unittest.skipUnless(HAVE, SKIP)
class TestCoreHook(unittest.TestCase):
    def test_request_is_added_to_the_outgoing_frames_only_while_armed_and_cleared_by_safe(self):
        from nimbus_core.core import NimbusCore
        from nimbus_pc.sender import Controls
        core = NimbusCore(); core.connect_sim()
        try:
            core.set_source("external"); core.set_allow_arm(True); core.set_external_controls(Controls(arm=True)); core.set_fc_hold(900, 1000, None)
            core.tick(); self.assertEqual((core.last_sent.aux5, core.last_sent.aux6), (900, 1000))
            core.set_external_controls(Controls(arm=False)); core.tick(); self.assertNotEqual(core.last_sent.aux5, 900)      # arm low: no request
            core.set_external_controls(Controls(arm=True)); core.tick(); self.assertEqual(core.last_sent.aux5, 900)
            core.safe_now(); self.assertIsNone(core.fc_hold_aux)
        finally:
            core.disconnect()


@unittest.skipUnless(HAVE and os.environ.get("DRONEFORGE_SLOW", "1") == "1", SKIP)
class TestScript(unittest.TestCase):
    def test_fc_hold_run_takes_off_holds_and_lands_in_the_simulator(self):
        sys.path.insert(0, os.path.expanduser("~/Music/DroneForge/tools_scripts"))
        import io, contextlib
        import fc_hold_run
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rc = fc_hold_run.main(["--sim", "--countdown", "1", "--hold", "3", "--hover-ref", "0.66", "--takeoff-timeout", "90"])
        out = buf.getvalue()
        self.assertEqual(rc, 0, out[-600:]); self.assertIn("HOLDING", out); self.assertIn("on the ground", out)

    def test_above_the_height_limit_the_fc_lands_slowly_not_a_disarm(self):
        sys.path.insert(0, os.path.expanduser("~/Music/DroneForge/tools_scripts"))
        import io, contextlib
        import fc_hold_run
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rc = fc_hold_run.main(["--sim", "--countdown", "1", "--hold", "30", "--height", "0.6", "--abort-height", "0.35", "--hover-ref", "0.66", "--takeoff-timeout", "90"])
        out = buf.getvalue()
        self.assertEqual(rc, 0, out[-600:]); self.assertIn("the FC LANDS slowly", out); self.assertIn("on the ground", out); self.assertNotIn(": SAFE", out)


if __name__ == "__main__":
    unittest.main()
