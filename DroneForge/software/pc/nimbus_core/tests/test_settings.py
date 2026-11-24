import os
import tempfile
import time
import unittest

from nimbus_core import settings as S
from nimbus_core.core import NimbusCore
from nimbus_pc.emulator import DongleEmulator


class TestSettings(unittest.TestCase):
    def test_validation_save_load_roundtrip_and_bad_files(self):
        p = os.path.join(tempfile.mkdtemp(), "s.json")
        self.assertEqual(S.load(p), S.FlightSettings())                       # missing file -> defaults
        s = S.FlightSettings(0.3, 0.33, 0.55, True); S.save(s, p)
        self.assertEqual(S.load(p), s)
        open(p, "w").write("{broken"); self.assertEqual(S.load(p), S.FlightSettings())
        for bad in (S.FlightSettings(3.0), S.FlightSettings(0.5, 0.9), S.FlightSettings(0.5, 0.45, 0.46)):
            self.assertTrue(bad.validate())
            with self.assertRaises(ValueError): S.save(bad, p)
        open(p, "w").write('{"hover_height": 9.0}'); self.assertEqual(S.load(p), S.FlightSettings())   # invalid values are not used

    def test_apply_and_core_gate(self):
        core = NimbusCore()
        au = core.autonomy
        self.assertEqual(au.set_settings(S.FlightSettings(0.3, 0.36, 0.6, True), persist=False), [])
        self.assertAlmostEqual(au.cfg.thr_hover0, 0.36); self.assertAlmostEqual(au.cfg.hover_height, 0.3); self.assertAlmostEqual(au.cfg.thr_max, 0.6)
        self.assertTrue(au.set_settings(S.FlightSettings(0.3, 0.5, 0.52), persist=False))   # cap too low: refused, previous kept
        self.assertAlmostEqual(au.cfg.thr_hover0, 0.36)

    def test_real_takeoff_needs_a_measured_hover_throttle_but_sim_does_not(self):
        emu = DongleEmulator(); emu.battery_dv = 160; emu.start(); core = NimbusCore()
        try:
            core.connect(emu.path); core.set_source("external"); core.set_allow_arm(True)
            for _ in range(200): core.tick(); time.sleep(0.02)
            ok, why = core.autonomy.preflight()
            self.assertFalse(ok); self.assertTrue(any("UNMEASURED" in w for w in why))
            ok, why = core.autonomy.preflight(for_ramp=True)               # the ramp itself must not be blocked by the measured-throttle rule
            self.assertFalse(any("UNMEASURED" in w for w in why))
            core.autonomy.set_settings(S.FlightSettings(0.3, 0.40, 0.7, True), persist=False)
            ok, why = core.autonomy.preflight()
            self.assertFalse(any("UNMEASURED" in w for w in why))
        finally:
            core.disconnect(); emu.stop()
        sim = NimbusCore(); sim.connect_sim(); sim.set_source("external"); sim.set_allow_arm(True)
        try:
            time.sleep(4)
            self.assertFalse(any("UNMEASURED" in w for w in sim.autonomy.preflight()[1]))
        finally:
            sim.disconnect()


if __name__ == "__main__":
    unittest.main()
