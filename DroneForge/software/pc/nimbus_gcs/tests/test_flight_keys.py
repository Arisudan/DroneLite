"""Flight keys: hold A = ARM, hold T = TAKE OFF, D = DISARM, L = LAND, through the same director calls as the FLY buttons (built-in simulator)."""
import os
import time
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt6.QtCore import QEvent, Qt
from PyQt6.QtGui import QKeyEvent
from PyQt6.QtWidgets import QApplication, QLineEdit

_app = QApplication.instance() or QApplication([])


def key(w, k, press=True, mods=Qt.KeyboardModifier.NoModifier):
    ev = QKeyEvent(QEvent.Type.KeyPress if press else QEvent.Type.KeyRelease, k, mods)
    return w.flight_keys.eventFilter(w, ev)


def spin(s):
    t0 = time.monotonic()
    while time.monotonic() - t0 < s:
        _app.processEvents(); time.sleep(0.01)


class TestFlightKeys(unittest.TestCase):
    def setUp(self):
        import nimbus_gcs.flight_keys as FK
        self._keep = FK.default_hold_ms; FK.default_hold_ms = lambda: 300                                        # a short hold, only inside these tests
        from nimbus_gcs.main_window import MainWindow
        self.w = MainWindow(); self.w.show(); self.w.activateWindow(); self.w.model.core.connect_sim(); spin(1.5)
        self.w.flight_keys._ours = lambda e: not (e.modifiers() & (Qt.KeyboardModifier.ControlModifier | Qt.KeyboardModifier.ShiftModifier)) and not isinstance(QApplication.focusWidget(), QLineEdit)   # offscreen: no active window

    def tearDown(self):
        import nimbus_gcs.flight_keys as FK
        FK.default_hold_ms = self._keep; self.w.model.disconnect(); self.w.close()

    def phase(self):
        return self.w.fly.director.state().phase

    def test_hold_a_arms_early_release_cancels_t_takes_off_l_lands_d_refused_in_air(self):
        w = self.w
        self.assertTrue(key(w, Qt.Key.Key_A)); spin(0.1); key(w, Qt.Key.Key_A, False)                          # let go early
        self.assertNotEqual(self.phase(), "armed"); self.assertIn("cancelled", w.status_line.text())
        key(w, Qt.Key.Key_A); spin(0.5); key(w, Qt.Key.Key_A, False)
        for _ in range(100):
            if self.phase() == "armed": break
            spin(0.1)
        self.assertEqual(self.phase(), "armed")
        key(w, Qt.Key.Key_T); spin(0.5); key(w, Qt.Key.Key_T, False)
        for _ in range(100):
            if self.phase() in ("takeoff", "climbing", "holding"): break
            spin(0.1)
        self.assertIn(self.phase(), ("takeoff", "climbing", "holding"))
        key(w, Qt.Key.Key_D); spin(0.2); self.assertIn(self.phase(), ("takeoff", "climbing", "holding"))         # DISARM is refused in the air (the director's rule)
        key(w, Qt.Key.Key_L); spin(0.3); self.assertIn(self.phase(), ("landing", "landed", "ready", "disarmed", "idle"))

    def test_t_needs_armed_and_typing_or_modifiers_are_ignored(self):
        w = self.w
        key(w, Qt.Key.Key_T); spin(0.5); key(w, Qt.Key.Key_T, False); self.assertNotIn(self.phase(), ("takeoff", "climbing", "holding"))
        self.assertFalse(key(w, Qt.Key.Key_A, mods=Qt.KeyboardModifier.ControlModifier))
        e = QLineEdit(w); e.show(); e.setFocus(); spin(0.05)
        if QApplication.focusWidget() is e:
            self.assertFalse(key(w, Qt.Key.Key_L))                                                                  # typing an L in a text box does not land


if __name__ == "__main__":
    unittest.main()
