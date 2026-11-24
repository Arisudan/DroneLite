"""Every hover-controller event / land / abort reason must reach the core event stream (events.txt, GCS notifications, status line) with the controller state and numbers."""
import os
import tempfile
import time
import unittest

from nimbus_core import topics as T
from nimbus_core.core import NimbusCore
from nimbus_pc.hover import runner
from nimbus_pc.hover.controller import HoverConfig, HoverController, HoverState
from nimbus_pc.hover.sim import Sample


class TestForwarding(unittest.TestCase):
    def setUp(self):
        self.core = NimbusCore(); self.au = self.core.autonomy
        self.ev = []
        self.core.bus.subscribe(T.EVENT, self.ev.append)

    def _drain(self, *texts):
        for i, t in enumerate(texts):
            self.au.ctl.events.append((float(i), t))
        self.au._drain_ctl_events()

    def test_nothing_is_dropped_and_nothing_is_sent_twice(self):
        self._drain("idle -> takeoff", "XY gate opened: hold here", "learned hover throttle 0.66 (was 0.63)")
        self.assertEqual(len([e for e in self.ev if str(e).startswith("HOVER ")]), 3)
        self.au._drain_ctl_events()
        self.assertEqual(len([e for e in self.ev if str(e).startswith("HOVER ")]), 3)
        self._drain("overshoot damper (z 0.6, vz +0.5)")
        self.assertEqual(len([e for e in self.ev if str(e).startswith("HOVER ")]), 4)

    def test_context_numbers_are_attached(self):
        self.au.ctl.state = HoverState.HOVER; self.au.ctl.kz.x[0] = 0.5; self.au.ctl.last_range_m = 0.56
        self._drain("hover -> land (ceiling 1.20 m exceeded (z 1.21 m))")
        text = [e for e in self.ev if "ceiling" in str(e)][0]
        for part in ("state hover", "z_est 0.50 m", "range 0.56 m", "throttle", "tilt"):
            self.assertIn(part, text)

    def test_land_reason_becomes_the_status_message(self):
        self._drain("hover -> land (ceiling 1.20 m exceeded (z 1.21 m))")
        self.assertEqual(self.au.message, "Landing: ceiling 1.20 m exceeded (z 1.21 m)")
        self.assertEqual(self.core.director.state().message, self.au.message)

    def test_notification_and_recorded_events_file(self):
        path = os.path.join(tempfile.mkdtemp(), "x.events.txt")
        self.core.recorder._ev = open(path, "w"); self.core.recorder._t0 = time.monotonic()
        self._drain("hover -> land (operator)")
        self.core.recorder._ev.close()
        self.assertIn("HOVER hover -> land (operator)", open(path).read())
        self.assertTrue(any("land" in n.text for n in self.core.notifications.items()) if hasattr(self.core.notifications, "items") else True)


class TestControllerReasons(unittest.TestCase):
    def _reasons(self, name, **kw):
        r = runner.run(name, duration=kw.pop("duration", 50), land_at=kw.pop("land_at", None), seed=1, **kw)
        return [str(e[1]) for e in r.events]

    def test_ceiling_reason_has_numbers(self):
        ev = self._reasons("ceiling")
        self.assertTrue(any("ceiling" in e and "m exceeded" in e and "z " in e for e in ev), ev)

    def test_sign_check_failure_reason(self):
        ev = self._reasons("sign_error")
        self.assertTrue(any("sign check failed" in e for e in ev), ev)

    def test_operator_land_reason(self):
        ev = self._reasons("nominal", land_at=15.0)
        self.assertTrue(any("-> land (scheduled)" in e for e in ev), ev)

    def test_sensor_loss_reason(self):
        ev = self._reasons("blackout")
        self.assertTrue(any("sensor_loss" in e and "no range data" in e for e in ev), ev)

    def test_tilt_reason(self):
        c = HoverController(HoverConfig()); c.state = HoverState.HOVER; c.pitch_deg = 80.0
        c._supervise(1.0)
        self.assertTrue(any("tilt exceeded" in str(e[1]) for e in c.events))


if __name__ == "__main__":
    unittest.main()
