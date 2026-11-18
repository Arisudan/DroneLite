"""MOTORS tab (3D drone view + control panel, layout after the older Flop ground station): arm at a tiny throttle (0..10 %) and check that every motor turns.

Design taken from the older Flop ground station (scripts/gcs/ui/motor_widget.py): a drawing of the aircraft from above with each rotor where it physically is, live output
gauges, spinning blades, and hard limits on the test. Differences: this drone is Betaflight, so there is no per-motor MAVLink test; ARMING at a very low throttle makes the FC
run all four motors at idle, and the FC reports the four motor outputs it sends over the radio (flight-mode text, see nimbus_pc.crsf.decode_flight_mode).

What it tells you: the FC's output for each motor (microseconds, 1000 = stopped) and, once you mark which motors you SEE turning, whether the fault is in the FC or outside it
(ESC signal wire, ESC, motor connector/phases, motor). It runs on the throttle-ramp logic (gyro calibration disarmed, arm at zero, soft start), limited to MAX_PCT, and
stops by itself after AUTO_STOP_S, on a tilt, on a lost link, or when the drone lifts.
"""
from __future__ import annotations

import time
from typing import List, Optional

from PyQt6.QtCore import QRectF, Qt, QTimer
from PyQt6.QtGui import QColor, QFont, QPainter, QPen
from PyQt6.QtWidgets import QApplication, QDoubleSpinBox, QGridLayout, QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget

from nimbus_core import topics as T
from . import theme
from .viz.drone_3d import Drone3DView

MAX_PCT = 10.0                     # hard ceiling of the test throttle (a hover needs about 70 %: this cannot lift the drone)
AUTO_STOP_S = 45.0
FRESH_S = 2.0                      # motor numbers older than this are shown as stale
# Betaflight default quad-X motor order (check it against YOUR wiring): number, label, position (right+, forward+)
MOTORS = [(1, "rear right", (+1.0, -1.0)), (2, "front right", (+1.0, +1.0)), (3, "rear left", (-1.0, -1.0)), (4, "front left", (-1.0, +1.0))]

def diagnose(outputs: Optional[List[int]], seen: List[Optional[bool]]) -> List[str]:
    """outputs: the four FC outputs in microseconds (None = no data); seen[i]: True = the pilot sees motor i+1 turning, False = not turning, None = not answered.
    Returns plain-language lines."""
    if outputs is None:
        return ["No motor numbers from the FC yet (not armed, no link, or the FC firmware without the motor read-out)."]
    out = []
    spread = max(outputs) - min(outputs)
    running = max(outputs) >= 1030
    if not running:
        out.append("The FC is sending no motor signal (all at 1000): arm first.")
    elif spread > 60:
        out.append(f"The FC's four outputs differ by {spread} us at a tiny throttle: it is correcting something (tilt or a wound-up integrator); level the drone and retry.")
    else:
        out.append(f"The FC sends the same signal to all four motors ({min(outputs)}..{max(outputs)} us).")
    for i, s in enumerate(seen):
        n = i + 1
        if s is False and running:
            if outputs[i] >= 1030 and spread <= 60:
                out.append(f"M{n}: the FC sends {outputs[i]} us (same as the others) but you see it NOT turning -> the fault is OUTSIDE the FC: M{n}'s ESC signal wire/solder pad, the ESC, "
                           f"the motor wires/connector, or the motor itself.")
            elif outputs[i] < 1030:
                out.append(f"M{n}: the FC itself sends only {outputs[i]} us: FC side (motor mapping/disabled).")
            else:
                out.append(f"M{n}: not turning while the FC sends {outputs[i]} us.")
    if running and all(s is True for s in seen):
        out.append("All four motors turn and the FC sends equal outputs: the motors are fine.")
    elif running and all(s is None for s in seen):
        out.append("Now look at each motor and mark below whether it turns.")
    return out


MAX_PCT_BAR = 10.0                 # the bar is full at the test ceiling (MAX_PCT)


class MotorButton(QPushButton):
    """Big checkable selection button for one motor (after the Flop GCS MotorTestPanel + MotorChannelBar): name, position, live output in us and percent, and an output bar.
    Selected = thicker accent border plus bold text; stale = '--' (reads without colour)."""
    def __init__(self, n: int, label: str) -> None:
        super().__init__()
        self.n, self.label, self.us, self.stale = n, label, 1000, True
        self.spinning = False; self.reason = ""
        self.setCheckable(True); self.setMinimumHeight(66); self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setToolTip(f"Press and HOLD to spin M{n} ({label}) alone; release to stop")

    def nextCheckState(self) -> None:                       # selection is set by the tab (press), never toggled by a click
        pass

    def set_output(self, us: Optional[int]) -> None:
        self.stale = us is None
        if us is not None:
            self.us = int(us)
        self.update()

    def percent(self) -> float:
        return max(0.0, min(100.0, (self.us - 1000) / 10.0))

    def paintEvent(self, _e) -> None:
        p = QPainter(self); p.setRenderHint(QPainter.RenderHint.Antialiasing)
        r = QRectF(self.rect()).adjusted(1, 1, -1, -1); sel = self.isChecked()
        p.setBrush(QColor(theme.HAIR if sel else theme.PANEL)); p.setPen(QPen(QColor(theme.ACC if sel else theme.HAIR2), 2.5 if sel else 1)); p.drawRoundedRect(r, 6, 6)
        p.setPen(QColor(theme.ACC if sel else theme.TXT)); f = QFont("Lato", 11); f.setBold(True); p.setFont(f)
        p.drawText(QRectF(r.x() + 10, r.y() + 6, r.width() - 20, 18), Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter, f"M{self.n}")
        f2 = QFont("Lato", 10); f2.setBold(sel); p.setFont(f2); p.setPen(QColor(theme.TXT if sel else theme.DIM))
        p.drawText(QRectF(r.x() + 10, r.y() + 6, r.width() - 20, 18), Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter, self.label)
        p.setFont(QFont("Noto Sans Mono", 9)); p.setPen(QColor(theme.DIM))
        txt = "SPINNING (release to stop)" if self.spinning else ("--" if self.stale else f"{self.us} us   {self.percent():.0f} %")
        if self.spinning:
            p.setPen(QColor(theme.ACC)); p.setFont(QFont("Noto Sans Mono", 9, QFont.Weight.Bold))
        elif not self.isEnabled():
            p.setPen(QColor(theme.DIM))
        p.drawText(QRectF(r.x() + 10, r.y() + 26, r.width() - 20, 16), Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter, txt)
        bar = QRectF(r.x() + 10, r.bottom() - 12, r.width() - 20, 6); p.setPen(Qt.PenStyle.NoPen); p.setBrush(QColor(theme.HAIR2)); p.drawRoundedRect(bar, 3, 3)
        if not self.stale and self.percent() > 0:
            p.setBrush(QColor(theme.ACC)); p.drawRoundedRect(QRectF(bar.x(), bar.y(), max(4.0, bar.width() * min(1.0, self.percent() / MAX_PCT_BAR)), 6), 3, 3)
        p.end()


class MotorTab(QWidget):
    def __init__(self, win) -> None:
        super().__init__()
        self.win, self.model = win, win.model
        self.outputs: Optional[List[int]] = None; self.thr_us: Optional[int] = None; self.t_out = 0.0
        self._t_start: Optional[float] = None
        l = QVBoxLayout(self); l.setContentsMargins(0, 0, 0, 0); l.setSpacing(16)
        warn = QLabel("THIS ARMS THE DRONE: all four motors spin at idle (props on). Hold or tether the drone, keep hands and cables clear, someone on SAFE.")
        warn.setWordWrap(True); warn.setStyleSheet(f"background:transparent;color:{theme.RED};border:1px solid {theme.RED};padding:8px 12px;border-radius:4px;font-weight:700"); l.addWidget(warn)
        warn2 = QLabel("THIS SPINS THE MOTOR YOU HOLD: props on, keep hands and cables clear, hold or tether the drone, release to stop.")
        warn2.setWordWrap(True); warn2.setStyleSheet(f"background:transparent;color:{theme.RED};border:1px solid {theme.RED};padding:8px 12px;border-radius:4px;font-weight:700"); l.addWidget(warn2)
        body = QHBoxLayout(); body.setSpacing(12); l.addLayout(body, 1)
        self.view = Drone3DView(); body.addWidget(self.view, 3)                             # left: the 3D drone (drag to orbit)
        self.view.motor_clicked.connect(self.select_motor)
        right = QVBoxLayout(); right.setSpacing(8); body.addLayout(right, 2); right.addStretch(0)               # right: controls, the pilot's check, the verdict
        row = QHBoxLayout(); row.setSpacing(8)
        row.addWidget(QLabel("Test throttle"))
        self.sp = QDoubleSpinBox(); self.sp.setRange(0.0, MAX_PCT); self.sp.setSingleStep(0.5); self.sp.setDecimals(1); self.sp.setSuffix(" %"); self.sp.setValue(1.0)
        self.sp.setToolTip(f"0 to {MAX_PCT:g} %. About 70 % is needed to hover, so this cannot lift the drone. Changes apply immediately while running.")
        self.sp.valueChanged.connect(self._on_throttle); row.addWidget(self.sp); row.addStretch(1)
        right.addLayout(row)
        self.btn_arm = QPushButton("ARM + SPIN"); self.btn_arm.setObjectName("primary"); self.btn_arm.setMinimumHeight(38); self.btn_arm.clicked.connect(self.on_arm); right.addWidget(self.btn_arm)
        self.btn_stop = QPushButton("STOP / DISARM"); self.btn_stop.setObjectName("ghost"); self.btn_stop.setMinimumHeight(38); self.btn_stop.clicked.connect(self.on_stop); right.addWidget(self.btn_stop)
        self.note = QLabel(""); self.note.setWordWrap(True); right.addWidget(self.note)
        self.status = QLabel("idle"); self.status.setWordWrap(True); right.addWidget(self.status)
        g = QGridLayout(); g.setHorizontalSpacing(8); g.setVerticalSpacing(4)
        self.mbtn: List[MotorButton] = []; self.selected = 0
        order = {4: (0, 0), 2: (0, 1), 3: (1, 0), 1: (1, 1)}                                  # laid out like the aircraft seen from above, front row on top
        for i, (n, lab, _pos) in enumerate(MOTORS):
            b = MotorButton(n, lab); b.pressed.connect(lambda n=n: self.on_press(n)); b.released.connect(self.on_release); self.mbtn.append(b)
            r, k = order[n]; g.addWidget(b, r, k)
        right.addLayout(g)
        right.addStretch(1)
        self._hold = QTimer(self); self._hold.setInterval(100); self._hold.timeout.connect(self._tick_hold)      # 10 Hz keep-alive while a button is held
        app = QApplication.instance()
        if app is not None:
            app.applicationStateChanged.connect(self._app_state); app.aboutToQuit.connect(self.stop_spin)
        win.model.core.bus.subscribe(T.TELEMETRY, self._on_tel)

    # --- data -----------------------------------------------------------------------------
    def _on_tel(self, msg) -> None:
        name, d = msg
        if name == "flight_mode" and "motors" in d:
            self.outputs, self.thr_us, self.t_out = list(d["motors"]), d.get("thr_us"), time.monotonic()

    # --- hold-to-spin ---------------------------------------------------------------------
    @property
    def mt(self):
        return self.model.core.motor_test

    def on_press(self, n: int) -> None:
        self.select_motor(n)
        why = self.mt.start(n)
        if why:
            self._note(why, "warning"); return
        self._hold.start(); self.refresh()

    def on_release(self) -> None:
        self.stop_spin("release")

    def stop_spin(self, reason: str = "app exit") -> None:
        self._hold.stop()
        self.mt.stop(reason)
        self.refresh()

    def _tick_hold(self) -> None:
        self.mt.refresh()
        if self.mt.state != "spinning":
            self._hold.stop()
            for b in self.mbtn:
                if b.isDown():
                    b.setDown(False)
        self.refresh()

    def _app_state(self, st) -> None:
        if st != Qt.ApplicationState.ApplicationActive:
            self.stop_spin("window focus lost")

    def hideEvent(self, e) -> None:
        self.stop_spin("tab change"); super().hideEvent(e)

    def running(self) -> bool:
        return self.model.core.ramp.phase in ("arming", "ramping")

    def select_motor(self, n: int) -> None:
        """Select motor n (0 = none): one button checked, ring in the 3D view."""
        self.selected = int(n); self.view.set_selected(self.selected)
        for b in self.mbtn:
            b.setChecked(b.n == self.selected)

    # --- actions --------------------------------------------------------------------------
    def _on_throttle(self, _v=None) -> None:
        if self.running():
            v = self.sp.value() / 100.0
            self.model.core.ramp.cfg.start_thr = self.model.core.ramp.cfg.cap_thr = v

    def _block_reason(self) -> str:
        m = self.model
        if not m.connected:
            return "Connect to the drone first."
        if m.core.autonomy.phase != "idle" or self.running():
            return "A flight or test is already running."
        return ""

    def _note(self, text: str, level: str = "ok") -> None:
        col = {"ok": theme.ACC, "warning": theme.AMB, "error": theme.RED}.get(level, theme.TXT)
        self.note.setText(text); self.note.setStyleSheet(f"padding:2px;color:{col}")

    def on_arm(self) -> None:
        m = self.model
        why = self._block_reason()
        if why:
            self._note(why, "warning"); return
        if m.source != "external":
            m.set_source("external"); self.win._set_combo(self.win.source_box, "external")
        if not m.allow_arm:
            m.set_allow_arm(True); self.win.chk_arm.blockSignals(True); self.win.chk_arm.setChecked(True); self.win.chk_arm.blockSignals(False)
        from nimbus_core.throttle_ramp import RampConfig
        v = min(self.sp.value(), MAX_PCT) / 100.0
        m.core.ramp.cfg = RampConfig(start_thr=v, cap_thr=v, rate_per_s=0.0, hold_at_cap_s=AUTO_STOP_S, lift_height_m=0.12, tilt_stop_deg=15.0)
        self.outputs = None
        if not m.core.ramp.start():
            self._note("Cannot start: arming must be allowed and the controls source external.", "error"); return
        self._note("Motor test started.", "ok")
        self._t_start = time.monotonic()

    def on_stop(self) -> None:
        self.model.core.ramp.stop("operator")

    # --- display --------------------------------------------------------------------------
    def refresh(self, *_a) -> None:
        run = self.running(); ramp = self.model.core.ramp; mt = self.mt; now = time.monotonic()
        fresh = self.outputs is not None and now - self.t_out < FRESH_S
        outs = list(self.outputs) if fresh else None
        if mt.state != "idle" and mt.readback is not None and now - mt.t_readback < FRESH_S and (not fresh or mt.t_readback > self.t_out):
            outs = list(mt.readback); fresh = True
        self.view.set_outputs(outs, not fresh)
        spinning = mt.state == "spinning"
        gate = mt.refusal()
        for i, b in enumerate(self.mbtn):
            b.set_output(outs[i] if fresh and outs and i < len(outs) else None)
            b.spinning = spinning and mt.motor == b.n
            en = (not gate) or b.isDown() or b.spinning
            if b.isEnabled() != en:
                b.setEnabled(en)
            b.setToolTip(f"Press and HOLD to spin M{b.n} ({b.label}) alone; release to stop" if en else f"M{b.n} locked: {gate}")
            b.update()
        why = self._block_reason()
        self.btn_arm.setEnabled(not why); self.btn_stop.setEnabled(run)
        if mt.state != "idle" or mt.status_text.startswith(("FC did not", "M")) and not gate:
            self._note(mt.status_text, "warning" if mt.unconfirmed else "ok")
        elif gate:
            self._note("Motor buttons locked: " + gate, "warning")
        elif self.note.text().startswith(("Connect to the drone first.", "Motor buttons locked", "A flight or test")):
            self._note("")
        if run:
            left = max(0.0, AUTO_STOP_S - (time.monotonic() - (self._t_start or time.monotonic())))
            self.status.setText(f"{ramp.phase.upper()}  throttle {ramp.thr * 100:.1f} %   (gyro calibration and arming take about 6 s first; stops itself in ~{left:.0f} s)")
        elif mt.state != "idle":
            self.status.setText(f"M{mt.motor} {'SPINNING (release to stop)' if spinning else 'stopping'}")
        else:
            res = ramp.result
            self.status.setText("idle" if not res else f"stopped: {res.get('reason')}")
