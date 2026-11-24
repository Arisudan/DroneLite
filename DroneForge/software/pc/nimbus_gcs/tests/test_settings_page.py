import os
import unittest
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt6.QtWidgets import QApplication, QCheckBox, QComboBox, QDoubleSpinBox, QMessageBox  # noqa: E402

from nimbus_core import settings as S                       # noqa: E402
from nimbus_core.director import Result                    # noqa: E402
from nimbus_gcs.main_window import apply_dark               # noqa: E402
from nimbus_gcs.settings_page import SettingsPage           # noqa: E402

_app = QApplication.instance() or QApplication([])
apply_dark(_app)


def _fake(refuse=False):
    au = SimpleNamespace(settings=S.FlightSettings())
    calls = []

    def apply(new, persist=True):
        calls.append(new)
        if refuse:
            return Result(False, "Refused: too high")
        au.settings = new
        return Result(True, "Settings applied")
    core = SimpleNamespace(autonomy=au, director=SimpleNamespace(apply_settings=apply))
    return SimpleNamespace(model=SimpleNamespace(core=core)), calls


class TestSettingsPage(unittest.TestCase):
    def setUp(self):
        for n in ("question", "information", "warning", "critical"):
            setattr(QMessageBox, n, staticmethod(lambda *a, **k: (_ for _ in ()).throw(AssertionError("dialog"))))

    def test_built_from_schema_no_checkbox(self):
        win, _ = _fake(); p = SettingsPage(win)
        self.assertEqual(p.findChildren(QCheckBox), [])
        for k in S.FIELD_KEYS:
            self.assertIn(k, p._widgets)
            self.assertIsInstance(p._widgets[k], (QDoubleSpinBox, QComboBox))

    def test_number_commits(self):
        win, calls = _fake(); p = SettingsPage(win)
        sp = p._widgets["hover_height"]; sp.setValue(1.2); sp.editingFinished.emit()
        self.assertEqual(len(calls), 1)
        self.assertAlmostEqual(calls[0].hover_height, 1.2)
        self.assertIn("Saved", p.status.text())

    def test_refusal_inline_and_reverts(self):
        win, calls = _fake(refuse=True); p = SettingsPage(win)
        old = win.model.core.autonomy.settings.hover_height
        sp = p._widgets["hover_height"]; sp.setValue(2.4); sp.editingFinished.emit()
        self.assertIn("Refused", p.status.text())
        self.assertAlmostEqual(sp.value(), old)

    def test_reset_group(self):
        win, calls = _fake(); p = SettingsPage(win)
        win.model.core.autonomy.settings = S.FlightSettings(thr_max=0.5)
        p._reset_group(2)
        self.assertEqual(win.model.core.autonomy.settings.thr_max, S.FlightSettings().thr_max)


def _cal_win(raw, sim=False, cells=4):
    win, calls = _fake()
    win.model.raw_vbat = lambda: raw
    win.model.is_sim = sim
    win.model.core.safety = SimpleNamespace(cfg=SimpleNamespace(cells=cells))
    return win, calls


class TestBatteryCalibration(unittest.TestCase):
    def test_calibrate_applies_and_reports(self):
        win, calls = _cal_win(13.8); p = SettingsPage(win)
        p.btn_cal.click()
        self.assertAlmostEqual(win.model.core.autonomy.settings.battery_scale, 4 * 4.16 / 13.8, delta=0.01)       # 1.206, shown as 'Factor 1.21'
        self.assertIn("Factor 1.21", p.status.text()); self.assertIn("13.8 V reads as", p.status.text())

    def test_refusals(self):
        for raw, sim in ((None, False), (11.0, False), (13.8, True)):
            win, calls = _cal_win(raw, sim); p = SettingsPage(win)
            p.btn_cal.click()
            self.assertEqual(calls, []); self.assertIn("Cannot calibrate", p.status.text())

    def test_reset(self):
        win, calls = _cal_win(13.8); p = SettingsPage(win)
        p.btn_cal.click(); p.btn_cal_reset.click()
        self.assertEqual(win.model.core.autonomy.settings.battery_scale, 1.0)


if __name__ == "__main__":
    unittest.main()
