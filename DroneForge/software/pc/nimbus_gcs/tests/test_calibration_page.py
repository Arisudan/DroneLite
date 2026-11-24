import math
import os
import time
import unittest
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt6.QtWidgets import QApplication, QCheckBox, QMessageBox  # noqa: E402

from nimbus_core import settings as S                       # noqa: E402
from nimbus_gcs.main_window import apply_dark               # noqa: E402
from nimbus_gcs import theme
from nimbus_gcs.calibration_page import CalibrationPage     # noqa: E402

_app = QApplication.instance() or QApplication([])
apply_dark(_app)


def _fake(deg=(0.2, 0.1), armed=False, batt=15.0, lq=90, goto=None):
    au = SimpleNamespace(settings=S.FlightSettings(), fc_armed=False, phase="idle")
    core = SimpleNamespace(autonomy=au, connected=True, state_source="live", source="idle", ramp=SimpleNamespace(phase="idle"),
                           state=SimpleNamespace(decoded={}), sent=[])
    core.set_source = lambda s: setattr(core, "source", s)
    core.set_external_controls = lambda c: core.sent.append(c)
    model = SimpleNamespace(core=core, is_sim=False, raw_vbat=lambda: batt)
    win = SimpleNamespace(model=model, goto=goto or (lambda *a, **k: True), workspaces={})
    return win


def _feed(p, deg=(0.2, 0.1), mode="STAB*", lq=90, batt=15.0):
    now = time.monotonic()
    for i in range(6):
        p.cal.observe("attitude", {"pitch_rad": math.radians(deg[0]), "roll_rad": math.radians(deg[1])}, now=now - 1.5 + i * 0.3)
    p.cal.observe("flight_mode", {"mode": mode}, now=now); p.cal.observe("link", {"up_lq": lq}, now=now)
    if batt is not None:
        p.cal.observe("battery", {"voltage_v": batt}, now=now)
    p.refresh()


class TestCalibrationPage(unittest.TestCase):
    def setUp(self):
        for n in ("question", "information", "warning", "critical"):
            setattr(QMessageBox, n, staticmethod(lambda *a, **k: (_ for _ in ()).throw(AssertionError("dialog"))))

    def test_builds_no_checkbox_no_battery_ui(self):
        p = CalibrationPage(_fake())
        self.assertEqual(p.findChildren(QCheckBox), [])
        self.assertEqual(p.btn_level.label(), "CALIBRATE LEVEL"); self.assertIn("does NOT arm", p.lbl_help.text())
        self.assertFalse(p.btn_level.is_ready()); self.assertFalse(hasattr(p, "btn_cal"))
        self.assertIn("SETTINGS > Battery", p.lbl_batt.text()); self.assertIn("1.00", p.lbl_batt.text())

    def test_tilt_colours_and_words(self):
        for deg, col, word in (((0.2, 0.1), theme.ACC, "LEVEL"), ((1.0, 0.0), theme.AMB, "SLIGHTLY"), ((0.0, 2.0), theme.RED, "TILTED")):
            p = CalibrationPage(_fake()); _feed(p, deg)
            self.assertIn(col, p.lbl_tilt.styleSheet()); self.assertIn(word, p.lbl_tilt.text())
        p = CalibrationPage(_fake()); _feed(p, (1.0, 0.0)); self.assertIn("pitch +1.0 deg", p.lbl_tilt.text())

    def test_stale_attitude_not_coloured(self):
        p = CalibrationPage(_fake()); p.cal.observe("attitude", {"pitch_rad": 0.0, "roll_rad": 0.0}, now=time.monotonic() - 30); p.refresh()
        self.assertIn("stale", p.lbl_tilt.text()); self.assertIn(theme.DIM, p.lbl_tilt.styleSheet())

    def test_button_enabled_when_ready_and_reasons_shown(self):
        p = CalibrationPage(_fake()); _feed(p)
        self.assertTrue(p.btn_level.is_ready()); self.assertEqual(p.why_level.text(), "")
        _feed(p, mode="STAB"); self.assertFalse(p.btn_level.is_ready()); self.assertIn("armed", p.why_level.text())
        _feed(p, lq=30); self.assertIn("Link quality", p.why_level.text())
        _feed(p, batt=None); self.assertTrue(p.btn_level.is_ready())   # battery was seen 0 s ago in the previous feed

    def test_level_activation_runs_and_shows_before_after(self):
        win = _fake(); p = CalibrationPage(win); _feed(p, (0.4, -0.2))
        p.cal._sleep = lambda d: None
        t = [time.monotonic()]
        p.cal._clock = lambda: (t.__setitem__(0, t[0] + 0.02), t[0])[1]
        orig = win.model.core.set_external_controls
        def send(c):
            orig(c); p.cal.observe("attitude", {"pitch_rad": math.radians(0.1), "roll_rad": math.radians(0.0)}, now=t[0])
            p.cal.observe("flight_mode", {"mode": "STAB*"}, now=t[0]); p.cal.observe("link", {"up_lq": 90}, now=t[0]); p.cal.observe("battery", {"voltage_v": 15}, now=t[0])
        win.model.core.set_external_controls = send
        for i in range(6):
            p.cal.observe("attitude", {"pitch_rad": math.radians(0.4), "roll_rad": math.radians(-0.2)}, now=t[0] - 0.1 * i)
        p.cal.observe("flight_mode", {"mode": "STAB*"}, now=t[0]); p.cal.observe("link", {"up_lq": 90}, now=t[0]); p.cal.observe("battery", {"voltage_v": 15}, now=t[0])
        p.btn_level.activated.emit(); p.cal._thread.join(10); p.refresh()
        self.assertIn("Before", p.res_level.text()); self.assertIn("After", p.res_level.text()); self.assertIn("Level calibrated", p.res_level.text())
        self.assertEqual(win.model.core.sent[-1].throttle, 0.0)

    def test_gyro_card(self):
        p = CalibrationPage(_fake()); self.assertFalse(p.btn_gyro.isEnabled()); self.assertIn("status", p.lbl_gyro.text())
        _feed(p); self.assertTrue(p.btn_gyro.isEnabled()); self.assertIn("at every ARM", p.lbl_gyro_help.text())
        p.btn_gyro.click(); self.assertTrue(p.cal.gyro_measuring()); self.assertIn("Measuring", p.lbl_gyro.text())
        p.cal._gyro_t0 -= 5
        for i in range(10):
            p.cal.observe("flight_mode", {"mode": "STAB*", "gyro_dps10": [20, 0, 0]}, now=time.monotonic())
        p.refresh(); self.assertIn("NEEDS RE-CAL", p.res_gyro.text()); self.assertIn("next ARM", p.res_gyro.text())

    def test_battery_pointer_goes_to_settings(self):
        calls = []
        p = CalibrationPage(_fake(goto=lambda *a, **k: calls.append(a) or True)); p.btn_batt.click()
        self.assertEqual(calls, [("SETTINGS", "Settings")])


if __name__ == "__main__":
    unittest.main()
