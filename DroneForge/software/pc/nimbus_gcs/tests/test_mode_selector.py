"""Flight-mode selector on the Fly page (offscreen Qt, stubs only)."""
import os
import unittest
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
from PyQt6.QtWidgets import QApplication, QCheckBox, QMessageBox

from nimbus_core.director import FlightState, Result
from nimbus_gcs import theme
from nimbus_gcs.fly_page import FlyPage, MODES


class StubDirector:
    def __init__(self, connected=True):
        self.connected, self.mode, self.calls = connected, "pos_hold", []

    def state(self):
        return SimpleNamespace(phase="ready" if self.connected else "disconnected", armed=False, height_m=None, target_m=0.5, mission_phase="safe", phase_elapsed_s=0.0,
                               flight_mode=self.mode)

    def preflight(self):
        return []

    def set_mode(self, m):
        self.calls.append(m)
        if not self.connected:
            return Result(False, "Not connected")
        self.mode = m
        return Result(True, "ok " + m)


def page(connected=True):
    st = SimpleNamespace(hover_height=0.5, ceiling_m=3.0)
    d = StubDirector(connected)
    win = SimpleNamespace(model=SimpleNamespace(core=SimpleNamespace(director=d, autonomy=SimpleNamespace(settings=st)), state=SimpleNamespace(decoded={})))
    return FlyPage(win), d


def snap(connected=True):
    return dict(connected=connected, battery_pct=None, battery_v=None, lq=None, rssi=None)


class TestModeSelector(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([]); theme.load_fonts(); cls.app.setStyleSheet(theme.qss())

    def test_two_buttons_call_director_and_mark_current(self):
        p, d = page(); p.refresh(snap())
        self.assertEqual([b.text().replace("● ", "") for b in p.mode_btns.values()], ["ALT HOLD", "POS HOLD"])
        self.assertTrue(p.mode_btns["pos_hold"].text().startswith("●")); self.assertFalse(p.mode_btns["alt_hold"].text().startswith("●"))
        self.assertIn("holds the height and the position", p.mode_text.text())
        for m, _l, expl in MODES:
            p.mode_btns[m].click(); self.assertEqual(d.calls[-1], m)
            p.refresh(snap())
            self.assertTrue(p.mode_btns[m].text().startswith("●"))
            self.assertEqual(sum(b.text().startswith("●") for b in p.mode_btns.values()), 1)
            self.assertIn(expl, p.mode_text.text())
        self.assertEqual(d.calls, ["alt_hold", "pos_hold"])

    def test_disabled_with_reason_when_not_connected(self):
        p, d = page(False); p.refresh(snap(False))
        self.assertTrue(all(not b.isEnabled() for b in p.mode_btns.values()))
        self.assertIn("Not connected", p.mode_text.text())

    def test_no_checkbox_and_no_dialog(self):
        p, d = page()
        self.assertEqual(p.findChildren(QCheckBox), [])
        saved = QMessageBox.question
        QMessageBox.question = staticmethod(lambda *a, **k: (_ for _ in ()).throw(AssertionError("dialog")))
        try:
            p.mode_btns["alt_hold"].click()
        finally:
            QMessageBox.question = saved
        self.assertEqual(d.calls, ["alt_hold"])


if __name__ == "__main__":
    unittest.main()
