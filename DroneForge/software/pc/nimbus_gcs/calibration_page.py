"""Calibration page (SETUP > Calibration): LEVEL (FC accelerometer, over the radio), GYRO REST CHECK and a pointer to the battery factor.
The FC does the calibrating; this page commands it and shows the result. No dialogs, no tick boxes; refusals appear inline and the buttons are disabled when not allowed.
Battery calibration lives only in SETTINGS > Battery."""
from __future__ import annotations

from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget

from . import theme
from .components import Card, HoldButton

from nimbus_core.calibration import Calibrator

_COL = {"green": theme.ACC, "amber": theme.AMB, "red": theme.RED, "grey": theme.DIM}
_WORD = {"green": "LEVEL", "amber": "SLIGHTLY TILTED", "red": "TILTED", "grey": "NO FRESH DATA"}
_VERD = {"OK": "green", "CAUTION": "amber", "NEEDS RE-CAL": "red"}
LEVEL_HELP = ("Drone flat on a checked-level surface, still, props on, battery connected, FC USB unplugged. "
              "This does NOT arm and the motors do not spin.")
GYRO_HELP = ("The FC calibrates the gyro by itself at every ARM (the start sequence 'gyro calibrated, countdown done'): the drone must be still then. "
             "This check only measures the gyro at rest for 3 s: it changes nothing on the FC.")


def _label(obj: str = "") -> QLabel:
    l = QLabel(""); l.setWordWrap(True)
    if obj:
        l.setObjectName(obj)
    return l


class CalibrationPage(QWidget):
    def __init__(self, win):
        super().__init__()
        self.win = win
        self.cal = Calibrator(win.model.core)
        root = QVBoxLayout(self); root.setContentsMargins(0, 0, 0, 0); root.setSpacing(16)

        f = Card("Level (accelerometer)"); v = f.body
        self.lbl_help = _label("dim"); self.lbl_help.setText(LEVEL_HELP)
        self.lbl_tilt = _label(); self.res_level = _label(); self.why_level = _label("dim")
        self.btn_level = HoldButton("CALIBRATE LEVEL", "", "check", hold_ms=None)
        self.btn_level.setFixedHeight(72)
        v.addWidget(self.lbl_help); v.addWidget(self.lbl_tilt); v.addWidget(self.btn_level); v.addWidget(self.why_level); v.addWidget(self.res_level)
        self.btn_level.activated.connect(self._level)
        root.addWidget(f)

        f = Card("Gyro rest check"); v = f.body
        h = self.lbl_gyro_help = _label("dim"); h.setText(GYRO_HELP)
        self.lbl_gyro = _label(); self.res_gyro = _label()
        self.btn_gyro = QPushButton("CHECK GYRO AT REST"); self.btn_gyro.setObjectName("primary")
        row = QHBoxLayout(); row.addWidget(self.btn_gyro); row.addStretch(1)
        v.addWidget(h); v.addLayout(row); v.addWidget(self.lbl_gyro); v.addWidget(self.res_gyro)
        self.btn_gyro.clicked.connect(lambda _=False: self._gyro())
        root.addWidget(f)

        f = Card("Battery"); v = f.body
        self.lbl_batt = _label()
        self.btn_batt = QPushButton("Go to SETTINGS > Battery")
        row = QHBoxLayout(); row.addWidget(self.btn_batt); row.addStretch(1)
        v.addWidget(self.lbl_batt); v.addLayout(row)
        self.btn_batt.clicked.connect(lambda _=False: self._goto_battery())
        root.addWidget(f); root.addStretch(1)

        self._timer = QTimer(self); self._timer.setInterval(500); self._timer.timeout.connect(self.refresh)
        self.refresh()

    # --- helpers ------------------------------------------------------------------------------
    @staticmethod
    def _say(lbl: QLabel, text: str, col: str) -> None:
        lbl.setText(text); lbl.setStyleSheet(f"color:{_COL[col]}; font-weight:600;")

    def _core(self):
        return self.win.model.core

    def _scale(self) -> float:
        try:
            return float(self._core().autonomy.settings.battery_scale)
        except (AttributeError, TypeError, ValueError):
            return 1.0

    # --- refresh ------------------------------------------------------------------------------
    def refresh(self) -> None:
        s = self.cal.level_status()
        if s["pitch"] is None:
            self._say(self.lbl_tilt, "Rest tilt: no attitude data yet", "grey")
        else:
            age = "" if s["age"] is None else f", {s['age']:.1f} s old"
            self._say(self.lbl_tilt, f"Rest tilt: pitch {s['pitch']:+.1f} deg, roll {s['roll']:+.1f} deg (level is within 0.5 deg): {_WORD[s['colour']]}"
                      + (f" (data is stale{age})" if s["stale"] else ""), s["colour"])
        busy = self.cal.running
        why = None if busy else self.cal.level_refusal()
        if busy:
            why = self.cal.progress or "Level calibration is running."
        self.btn_level.set_enabled_with_reason(why is None, why or "")
        self.why_level.setText(("" if why is None else why) if not busy else "")
        r = self.cal.level_result
        if busy:
            self._say(self.res_level, self.cal.progress, "amber")
        elif r:
            self._say(self.res_level, r["text"], "green" if r["ok"] else "red" if r["after"] is not None else "amber")
        else:
            self.res_level.setText("")

        g = self.cal.gyro_poll()
        measuring = self.cal.gyro_measuring()
        gwhy = None if measuring else self.cal.gyro_refusal()
        self.btn_gyro.setEnabled(gwhy is None and not measuring and not busy)
        self.lbl_gyro.setText("Measuring for 3 s: keep the drone still..." if measuring else (gwhy or ""))
        if g and not measuring:
            self._say(self.res_gyro, g["text"] + ("  Needs a new calibration: it happens by itself at the next ARM (keep the drone still then)."
                                                  if g.get("verdict") == "NEEDS RE-CAL" else ""), _VERD.get(g.get("verdict", ""), "red"))
        elif not measuring:
            self.res_gyro.setText("")
        f = self._scale()
        self.lbl_batt.setText(f"Battery calibration is only in SETTINGS > Battery. Current factor: {f:.2f}.")

    # --- actions ------------------------------------------------------------------------------
    def _level(self) -> None:
        ok, msg = self.cal.calibrate_level()
        if not ok:
            self._say(self.res_level, msg, "red")
        self.refresh()

    def _gyro(self) -> None:
        ok, msg = self.cal.start_gyro_check()
        if not ok:
            self._say(self.res_gyro, msg, "red")
        self.refresh()

    def _goto_battery(self) -> None:
        win = self.win
        win.goto("SETTINGS", "Settings")
        try:
            ws = win.workspaces["SETTINGS"]; page = ws.build(ws.index_of("Settings")); page.nav.select("Battery")
        except Exception:                                    # noqa: BLE001
            pass

    def showEvent(self, e):
        self._timer.start(); self.refresh(); super().showEvent(e)

    def hideEvent(self, e):
        self._timer.stop(); super().hideEvent(e)
