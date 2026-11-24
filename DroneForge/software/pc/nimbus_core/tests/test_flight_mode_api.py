import unittest

from nimbus_core.core import NimbusCore
from nimbus_core.director import FlightState


class TestSetMode(unittest.TestCase):
    def setUp(self):
        self.core = NimbusCore(); self.d = self.core.director

    def tearDown(self):
        self.core.disconnect()

    def test_refused_when_not_connected(self):
        r = self.d.set_mode("alt_hold")
        self.assertFalse(r.ok); self.assertIn("Not connected", r.message)
        self.assertEqual(self.d.state().flight_mode, "pos_hold")

    def test_default_set_unknown_and_event(self):
        self.core.connect_sim(); ev = []
        from nimbus_core import topics as T
        self.core.bus.subscribe(T.EVENT, ev.append)
        self.assertEqual(self.d.state().flight_mode, "pos_hold")
        self.assertEqual(FlightState.__dataclass_fields__["flight_mode"].default, "pos_hold")
        r = self.d.set_mode("alt_hold")
        self.assertTrue(r.ok); self.assertIn("ALT HOLD", r.message)
        self.assertEqual(self.d.state().flight_mode, "alt_hold")
        self.assertEqual(self.core.autonomy.ctl.flight_mode, "alt_hold")
        bad = self.d.set_mode("acro")
        self.assertFalse(bad.ok); self.assertIn("Unknown flight mode", bad.message)
        self.assertEqual(self.d.state().flight_mode, "alt_hold")
        self.assertTrue(self.d.set_mode("stabilize").ok); self.assertTrue(self.d.set_mode("pos_hold").ok)
        self.assertIn("FLIGHT MODE -> alt_hold", ev)

    def test_not_saved_and_reset_after_flight(self):
        from nimbus_core import settings as S
        self.core.connect_sim(); self.d.set_mode("stabilize")
        self.assertFalse(hasattr(S.FlightSettings(), "flight_mode"))
        au = self.core.autonomy
        au.ctl.state = type(au.ctl.state).DONE; au.phase = "flying"
        au.controls(0.0)                                   # a finished flight drops back to idle
        self.assertEqual(self.d.state().flight_mode, "pos_hold")


if __name__ == "__main__":
    unittest.main()
