"""SETUP > MOTOR TEST, redesigned 2026-11-24 after the DRONE_GCS actuator workspace (DRONE_SRC/DRONE_GCS/scripts/gcs/ui/motor_widget.py): the same three-card idea
(the aircraft drawn top-down with each rotor where it really is | one output bar per motor | the bench test panel with its safety gates), drawn in the
DroneForge theme (greys + one accent, amber = caution, red only for STOP ALL), for a Betaflight FC over ELRS.

What it drives (nothing new on the FC side):
  * one motor at a time: nimbus_core.motor_test (MSP_SET_MOTOR over the radio, FC DISARMED only, hard ceiling 1100 us = 10 %, every press stops by itself after 2.5 s,
    a 10 Hz keep-alive while held, stop = zeros resent until the FC's motor read-back shows idle). HOLD TO SPIN, or SEQ 1-4 (each motor 1.5 s in turn).
  * all four motors: ARM + SPIN (nimbus_core.throttle_ramp: gyro calibration, arm, all motors at the test throttle; stops by itself after 45 s, on tilt / lift / link loss).
Gates (shown as the GATES list, enforced again in the core): live link with LQ >= 50 %, FC disarmed (for the single-motor test), not in a flight, and a SAFETY UNLOCK
that the pilot holds for 1 s (drone held or tethered, hands clear; props stay on) and that lapses by itself after 60 s. No check boxes, no dialogs: inline status lines.
Motor numbering and spin sense = Betaflight quad-X as on this drone (same table as viz/drone_3d.py): M1 rear right CCW, M2 front right CW, M3 rear left CW, M4 front left CCW.
The 3D version restored on 2026-11-23 is archived in archive/gcs_removed_2026-11-23/motor_tab_3d_restored.py."""
from __future__ import annotations

import math
import time
from typing import List, Optional

from PyQt6.QtCore import QPointF, QRectF, Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QBrush, QColor, QPainter, QPainterPath, QPen, QPolygonF, QRadialGradient
from PyQt6.QtWidgets import QApplication, QFrame, QGridLayout, QHBoxLayout, QLabel, QPushButton, QSizePolicy, QVBoxLayout, QWidget

from nimbus_core import topics as T
from . import theme

MAX_PCT = 10.0                     # hard ceiling of the test throttle (a hover needs about 70 %: this cannot lift the drone); the core clamps again (1100 us)
AUTO_STOP_S = 45.0                 # ARM + SPIN stops by itself
FRESH_S = 2.0                      # motor numbers older than this are shown as stale
UNLOCK_S = 60.0                    # the safety unlock lapses by itself
UNLOCK_HOLD_MS = 1000
SEQ_ON_S, SEQ_GAP_S = 1.5, 0.6     # SEQ 1-4: each motor this long, then a pause
STEP_PCT = 0.5
# Betaflight quad-X as on this drone: number -> (number, short label, long label, (right+, forward+), spin sense +1 = clockwise seen from above)
MOTORS = [(1, "RR", "rear right", (+1.0, -1.0), -1), (2, "FR", "front right", (+1.0, +1.0), +1),
          (3, "RL", "rear left", (-1.0, -1.0), +1), (4, "FL", "front left", (-1.0, +1.0), -1)]
_MOTOR = {m[0]: m for m in MOTORS}


def _c(tok: str, a: int = 255) -> QColor:
    c = QColor(tok); c.setAlpha(a); return c


def pct_of(us: int) -> float:
    return max(0.0, min(100.0, (us - 1000) / 10.0))


def diagnose(outputs: Optional[List[int]], seen: List[Optional[bool]]) -> List[str]:
    """outputs: the four FC outputs in us (None = no data); seen[i]: True = the pilot sees motor i+1 turning, False = not turning, None = not answered. Plain lines."""
    if outputs is None:
        return ["No motor numbers from the FC yet (not armed, no link, or the FC firmware without the motor read-out)."]
    out = []; spread = max(outputs) - min(outputs); running = max(outputs) >= 1030
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
                out.append(f"M{n}: the FC sends {outputs[i]} us (same as the others) but you see it NOT turning -> the fault is OUTSIDE the FC: M{n}'s ESC signal wire/solder pad, "
                           f"the ESC, the motor wires/connector, or the motor itself.")
            elif outputs[i] < 1030:
                out.append(f"M{n}: the FC itself sends only {outputs[i]} us: FC side (motor mapping/disabled).")
            else:
                out.append(f"M{n}: not turning while the FC sends {outputs[i]} us.")
    if running and all(s is True for s in seen):
        out.append("All four motors turn and the FC sends equal outputs: the motors are fine.")
    return out


# ---------------------------------------------------------------------------------------------------------------------------------------------------- frame geometry
class FrameView(QWidget):
    """The aircraft from above: tapered arms, the body with the O4 camera at the front (accent lights front, dim lights rear), each rotor where it is,
    props that turn at a rate proportional to the output, the spin sense as an arc arrow, a caption per rotor. Click a rotor to select it."""
    motor_clicked = pyqtSignal(int)

    def __init__(self) -> None:
        super().__init__()
        self.setMinimumSize(250, 300); self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.outputs: Optional[List[int]] = None; self.stale = True; self.selected = 0; self.spinning = 0
        self._ang = {n: 30.0 * n for n in _MOTOR}; self._t = time.monotonic(); self._hit = {}
        self._anim = QTimer(self); self._anim.setInterval(33); self._anim.timeout.connect(self._step)

    # ---- API
    def set_outputs(self, outs: Optional[List[int]], stale: bool) -> None:
        self.outputs, self.stale = (list(outs) if outs else None), bool(stale or not outs); self.update()

    def set_selected(self, n: int) -> None:
        self.selected = int(n); self.update()

    def set_spinning(self, n: int) -> None:
        self.spinning = int(n); self.update()

    def us(self, n: int) -> int:
        return self.outputs[n - 1] if (self.outputs and not self.stale and len(self.outputs) >= n) else 1000

    def showEvent(self, e) -> None:
        self._t = time.monotonic(); self._anim.start(); super().showEvent(e)

    def hideEvent(self, e) -> None:
        self._anim.stop(); super().hideEvent(e)

    def _step(self) -> None:
        now = time.monotonic(); dt = min(0.1, now - self._t); self._t = now; moving = False
        for n, (_i, _l, _ll, _p, sense) in _MOTOR.items():
            pct = pct_of(self.us(n))
            if pct > 0.3:
                self._ang[n] = (self._ang[n] + sense * dt * (240.0 + 90.0 * pct)) % 360.0; moving = True        # deg/s, clockwise = +
        if moving: self.update()

    # ---- geometry
    def _geo(self):
        W, H = self.width(), self.height(); cx, cy = W / 2.0, H / 2.0 + 6
        arm = min(W * 0.27, H * 0.24); rr = arm * 0.42
        ctr = {n: QPointF(cx + p[0] * arm, cy - p[1] * arm) for n, (_i, _l, _ll, p, _s) in _MOTOR.items()}
        return cx, cy, arm, rr, ctr

    def click_at(self, pt: QPointF) -> int:
        _cx, _cy, _arm, rr, ctr = self._geo()
        for n, c in ctr.items():
            if math.hypot(pt.x() - c.x(), pt.y() - c.y()) <= rr * 1.25:
                self.motor_clicked.emit(n); return n
        return 0

    def mousePressEvent(self, e) -> None:
        self.click_at(e.position())

    # ---- paint
    def paintEvent(self, _e) -> None:
        p = QPainter(self); p.setRenderHint(QPainter.RenderHint.Antialiasing)
        cx, cy, arm, rr, ctr = self._geo()
        p.setPen(QPen(_c(theme.HAIR, 90), 1)); p.setBrush(Qt.BrushStyle.NoBrush)                    # faint polar grid behind the aircraft
        for k in (1, 2, 3): p.drawEllipse(QPointF(cx, cy), arm * 0.62 * k, arm * 0.62 * k)
        p.drawLine(QPointF(cx - arm * 1.9, cy), QPointF(cx + arm * 1.9, cy)); p.drawLine(QPointF(cx, cy - arm * 1.9), QPointF(cx, cy + arm * 1.9))
        for c in ctr.values(): self._arm(p, QPointF(cx, cy), c, arm)
        self._body(p, cx, cy, arm)
        self._hit = {}
        for n, c in ctr.items(): self._rotor(p, n, c, rr)
        self._front(p, cx, cy, arm)
        p.end()

    def _arm(self, p: QPainter, a: QPointF, b: QPointF, arm: float) -> None:
        dx, dy = b.x() - a.x(), b.y() - a.y(); L = math.hypot(dx, dy) or 1.0; nx, ny = -dy / L, dx / L
        w0, w1 = arm * 0.10, arm * 0.055
        poly = QPolygonF([QPointF(a.x() + nx * w0, a.y() + ny * w0), QPointF(b.x() + nx * w1, b.y() + ny * w1),
                          QPointF(b.x() - nx * w1, b.y() - ny * w1), QPointF(a.x() - nx * w0, a.y() - ny * w0)])
        p.setPen(QPen(_c(theme.HAIR2), 1)); p.setBrush(_c(theme.HAIR)); p.drawPolygon(poly)

    def _body(self, p: QPainter, cx: float, cy: float, arm: float) -> None:
        bw, bh = arm * 0.62, arm * 0.98; r = QRectF(cx - bw / 2, cy - bh / 2, bw, bh)
        p.setPen(QPen(_c(theme.HAIR2), 1.2)); p.setBrush(_c(theme.PANEL)); p.drawRoundedRect(r, bw * 0.28, bw * 0.28)
        p.setPen(QPen(_c(theme.HAIR), 1)); p.setBrush(_c(theme.BG)); p.drawRoundedRect(r.adjusted(bw * 0.18, bh * 0.52, -bw * 0.18, -bh * 0.12), 4, 4)      # battery strap
        cam = QPointF(cx, r.top() + bh * 0.22); cr = bw * 0.2                                                                                  # the O4 camera
        p.setPen(QPen(_c(theme.HAIR2), 1)); p.setBrush(_c(theme.BG)); p.drawEllipse(cam, cr, cr)
        g = QRadialGradient(cam, cr * 0.7); g.setColorAt(0, _c(theme.ACC, 160)); g.setColorAt(1, _c(theme.BG, 0))
        p.setPen(Qt.PenStyle.NoPen); p.setBrush(QBrush(g)); p.drawEllipse(cam, cr * 0.6, cr * 0.6)
        for sx in (-1, 1):
            p.setBrush(_c(theme.ACC)); p.drawEllipse(QPointF(cx + sx * bw * 0.36, r.top() + bh * 0.06), 3.2, 3.2)                    # front lights
            p.setBrush(_c(theme.FAINT)); p.drawEllipse(QPointF(cx + sx * bw * 0.36, r.bottom() - bh * 0.06), 3.2, 3.2)               # rear lights

    def _front(self, p: QPainter, cx: float, cy: float, arm: float) -> None:
        y = cy - arm * 1.62; p.setPen(Qt.PenStyle.NoPen); p.setBrush(_c(theme.ACC))
        p.drawPolygon(QPolygonF([QPointF(cx, y - 9), QPointF(cx - 8, y + 3), QPointF(cx + 8, y + 3)]))
        p.setPen(_c(theme.DIM)); p.setFont(theme.f_label(10)); p.drawText(QRectF(cx - 60, y + 5, 120, 16), Qt.AlignmentFlag.AlignHCenter, "FRONT")

    def _rotor(self, p: QPainter, n: int, c: QPointF, rr: float) -> None:
        _i, lab, _ll, pos, sense = _MOTOR[n]; us = self.us(n); pct = pct_of(us)
        sel = n == self.selected; spin = n == self.spinning or pct > 0.3
        g = QRadialGradient(c, rr); g.setColorAt(0, _c(theme.PANEL, 230)); g.setColorAt(1, _c(theme.BG, 160))
        p.setPen(Qt.PenStyle.NoPen); p.setBrush(QBrush(g)); p.drawEllipse(c, rr, rr)
        if spin:                                                                                   # blur ring while turning
            p.setPen(QPen(_c(theme.ACC, 70), rr * 0.18)); p.setBrush(Qt.BrushStyle.NoBrush); p.drawEllipse(c, rr * 0.82, rr * 0.82)
        pen = QPen(_c(theme.ACC) if sel else _c(theme.HAIR2), 2.2 if sel else 1); pen.setStyle(Qt.PenStyle.SolidLine if sel else Qt.PenStyle.DashLine)
        p.setPen(pen); p.setBrush(Qt.BrushStyle.NoBrush); p.drawEllipse(c, rr, rr)
        p.save(); p.translate(c); p.rotate(self._ang[n])                                          # two blades
        blade = QPainterPath(); blade.moveTo(0, -rr * 0.06); blade.cubicTo(rr * 0.35, -rr * 0.16, rr * 0.8, -rr * 0.1, rr * 0.92, 0)
        blade.cubicTo(rr * 0.8, rr * 0.07, rr * 0.35, rr * 0.08, 0, rr * 0.06)
        p.setPen(Qt.PenStyle.NoPen); p.setBrush(_c(theme.DIM, 120 if spin else 200)); p.drawPath(blade); p.rotate(180); p.drawPath(blade)
        p.restore()
        p.setPen(QPen(_c(theme.HAIR2), 1)); p.setBrush(_c(theme.PANEL)); p.drawEllipse(c, rr * 0.24, rr * 0.24)                    # hub with the number
        p.setPen(_c(theme.ACC if sel else theme.TXT)); f = theme.f_body(12); f.setBold(True); p.setFont(f)
        p.drawText(QRectF(c.x() - rr * 0.4, c.y() - 9, rr * 0.8, 18), Qt.AlignmentFlag.AlignCenter, f"M{n}")
        out = math.atan2(-pos[1], pos[0]); span = math.radians(70)                                # spin-sense arc on the outer side
        R = rr * 1.16; a0 = out - span / 2; a1 = out + span / 2
        arc = QPainterPath()
        for k in range(17):
            a = a0 + (a1 - a0) * k / 16; pt = QPointF(c.x() + R * math.cos(a), c.y() + R * math.sin(a))
            arc.moveTo(pt) if k == 0 else arc.lineTo(pt)
        p.setPen(QPen(_c(theme.ACC if spin else theme.FAINT), 1.6)); p.setBrush(Qt.BrushStyle.NoBrush); p.drawPath(arc)
        ah = a1 if sense > 0 else a0; d = 1 if sense > 0 else -1                                  # screen y is down: clockwise = increasing angle
        tip = QPointF(c.x() + R * math.cos(ah), c.y() + R * math.sin(ah)); tan = (-math.sin(ah) * d, math.cos(ah) * d); nrm = (math.cos(ah), math.sin(ah)); s = 6
        p.setPen(Qt.PenStyle.NoPen); p.setBrush(_c(theme.ACC if spin else theme.FAINT))
        p.drawPolygon(QPolygonF([QPointF(tip.x() + tan[0] * s, tip.y() + tan[1] * s), QPointF(tip.x() - tan[0] * s * 0.4 + nrm[0] * s * 0.7, tip.y() - tan[1] * s * 0.4 + nrm[1] * s * 0.7),
                                 QPointF(tip.x() - tan[0] * s * 0.4 - nrm[0] * s * 0.7, tip.y() - tan[1] * s * 0.4 - nrm[1] * s * 0.7)]))
        bw, bh = 132, 36; above = pos[1] > 0                                                         # caption above a front rotor / below a rear one, kept inside the view
        bx = min(max(4.0, c.x() - bw / 2), self.width() - bw - 4); by = c.y() - rr * 1.42 - bh if above else c.y() + rr * 1.42
        by = min(max(4.0, by), self.height() - bh - 4); box = QRectF(bx, by, bw, bh)
        p.setPen(QPen(_c(theme.ACC) if sel else _c(theme.HAIR), 1)); p.setBrush(_c(theme.BG, 210)); p.drawRoundedRect(box, 6, 6)
        p.setFont(theme.f_label(10)); p.setPen(_c(theme.TXT))
        p.drawText(box.adjusted(8, 2, -8, -bh / 2), Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter, f"M{n}  {lab}")
        p.setPen(_c(theme.DIM)); p.drawText(box.adjusted(8, 2, -8, -bh / 2), Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter, "CW" if sense > 0 else "CCW")
        p.setFont(theme.f_mono(11)); p.setPen(_c(theme.FAINT if self.stale else theme.TXT))
        p.drawText(box.adjusted(8, bh / 2, -8, -2), Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter, "--" if self.stale else f"{us} µs · {pct:.0f} %")
        self._hit[n] = (c, rr)


# ------------------------------------------------------------------------------------------------------------------------------------------------------- output bars
class ChannelBar(QWidget):
    """One motor's FC output, 1000..2000 us, with the test ceiling (1100 us) marked. Stale = dashed outline and '--' (reads without colour)."""
    def __init__(self, n: int) -> None:
        super().__init__()
        self.n = n; self.us = 1000; self.stale = True; self.active = False
        self.setMinimumSize(40, 200); self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

    def set_output(self, us: Optional[int], active: bool = False) -> None:
        self.stale = us is None; self.us = 1000 if us is None else int(us); self.active = active; self.update()

    def paintEvent(self, _e) -> None:
        p = QPainter(self); p.setRenderHint(QPainter.RenderHint.Antialiasing)
        W, H = self.width(), self.height(); _i, lab, _ll, _p, sense = _MOTOR[self.n]
        f = theme.f_body(13); f.setBold(True); p.setFont(f); p.setPen(_c(theme.ACC if self.active else theme.TXT))
        p.drawText(QRectF(0, 0, W, 22), Qt.AlignmentFlag.AlignHCenter, f"M{self.n}")
        tw = min(34.0, W * 0.5); track = QRectF((W - tw) / 2, 28, tw, H - 28 - 62)
        pen = QPen(_c(theme.HAIR2), 1); pen.setStyle(Qt.PenStyle.DashLine if self.stale else Qt.PenStyle.SolidLine)
        p.setPen(pen); p.setBrush(_c(theme.BG)); p.drawRoundedRect(track, 6, 6)
        if not self.stale:
            fr = pct_of(self.us) / 100.0; fh = max(0.0, track.height() * fr - 6)
            if fh > 0:
                p.setPen(Qt.PenStyle.NoPen); p.setBrush(_c(theme.AMB if fr > 0.75 else theme.ACC, 230))
                p.drawRoundedRect(QRectF(track.x() + 3, track.bottom() - 3 - fh, track.width() - 6, fh), 4, 4)
        y = track.bottom() - track.height() * MAX_PCT / 100.0                                         # the test ceiling
        p.setPen(QPen(_c(theme.ACC, 150), 1, Qt.PenStyle.DashLine)); p.drawLine(QPointF(track.x() - 6, y), QPointF(track.right() + 6, y))
        p.setPen(QPen(_c(theme.HAIR, 160), 1, Qt.PenStyle.DotLine)); ym = track.center().y(); p.drawLine(QPointF(track.x() + 2, ym), QPointF(track.right() - 2, ym))
        p.setFont(theme.f_mono(12)); p.setPen(_c(theme.FAINT if self.stale else theme.TXT))
        p.drawText(QRectF(0, H - 58, W, 18), Qt.AlignmentFlag.AlignHCenter, "--" if self.stale else f"{self.us} µs")
        p.setFont(theme.f_mono(11)); p.setPen(_c(theme.DIM)); p.drawText(QRectF(0, H - 40, W, 16), Qt.AlignmentFlag.AlignHCenter, "" if self.stale else f"{pct_of(self.us):.1f} %")
        p.setFont(theme.f_label(9)); p.setPen(_c(theme.FAINT)); p.drawText(QRectF(0, H - 22, W, 16), Qt.AlignmentFlag.AlignHCenter, f"{lab} · {'CW' if sense > 0 else 'CCW'}")
        p.end()


# ----------------------------------------------------------------------------------------------------------------------------------------------------- test controls
class ThrottleSlider(QWidget):
    """0..MAX_PCT % in 0.1 % steps: drag, click, wheel or arrow keys; ticks every 2.5 %."""
    changed = pyqtSignal(float)

    def __init__(self) -> None:
        super().__init__()
        self._v = 7.5; self.setFixedHeight(38); self.setMinimumWidth(110); self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

    def value(self) -> float:
        return self._v

    def setValue(self, v: float) -> None:
        v = round(max(0.0, min(MAX_PCT, float(v))), 1)
        if v != self._v: self._v = v; self.update(); self.changed.emit(v)

    def _x(self, v: float) -> float:
        return 12 + (self.width() - 24) * v / MAX_PCT

    def _from_x(self, x: float) -> float:
        return (x - 12) / max(1.0, self.width() - 24) * MAX_PCT

    def mousePressEvent(self, e) -> None:
        self.setValue(self._from_x(e.position().x()))

    def mouseMoveEvent(self, e) -> None:
        if e.buttons(): self.setValue(self._from_x(e.position().x()))

    def wheelEvent(self, e) -> None:
        self.setValue(self._v + (STEP_PCT if e.angleDelta().y() > 0 else -STEP_PCT))

    def keyPressEvent(self, e) -> None:
        k = e.key()
        if k in (Qt.Key.Key_Right, Qt.Key.Key_Up): self.setValue(self._v + STEP_PCT)
        elif k in (Qt.Key.Key_Left, Qt.Key.Key_Down): self.setValue(self._v - STEP_PCT)
        else: super().keyPressEvent(e)

    def paintEvent(self, _e) -> None:
        p = QPainter(self); p.setRenderHint(QPainter.RenderHint.Antialiasing); y = 13.0
        p.setPen(Qt.PenStyle.NoPen); p.setBrush(_c(theme.HAIR)); p.drawRoundedRect(QRectF(12, y - 3, self.width() - 24, 6), 3, 3)
        x = self._x(self._v); p.setBrush(_c(theme.ACC)); p.drawRoundedRect(QRectF(12, y - 3, max(0.0, x - 12), 6), 3, 3)
        p.setPen(QPen(_c(theme.FAINT), 1)); p.setFont(theme.f_mono(9))
        for k in range(0, int(MAX_PCT / 2.5) + 1):
            tx = self._x(k * 2.5); p.drawLine(QPointF(tx, y + 7), QPointF(tx, y + 11)); p.drawText(QRectF(tx - 20, y + 11, 40, 13), Qt.AlignmentFlag.AlignHCenter, f"{k * 2.5:g}")
        p.setPen(QPen(_c(theme.ACC), 2)); p.setBrush(_c(theme.PANEL)); p.drawEllipse(QPointF(x, y), 8, 8)
        if self.hasFocus(): p.setPen(QPen(_c(theme.ACC, 90), 4)); p.setBrush(Qt.BrushStyle.NoBrush); p.drawEllipse(QPointF(x, y), 11, 11)
        p.end()


class HoldPill(QPushButton):
    """Press and hold for `hold_ms`: a bar fills; `held` fires only after the full hold (the safety unlock; no check box)."""
    held = pyqtSignal()

    def __init__(self, text: str, hold_ms: int = UNLOCK_HOLD_MS) -> None:
        super().__init__(text)
        self.hold_ms = hold_ms; self._t0: Optional[float] = None; self.progress = 0.0; self.setMinimumHeight(40)
        self._tm = QTimer(self); self._tm.setInterval(16); self._tm.timeout.connect(self._tick)
        self.pressed.connect(self._down); self.released.connect(self._up)

    def _down(self) -> None:
        self._t0 = time.monotonic(); self._tm.start()

    def _up(self) -> None:
        self._t0 = None; self.progress = 0.0; self._tm.stop(); self.update()

    def _tick(self) -> None:
        if self._t0 is None: return
        self.progress = min(1.0, (time.monotonic() - self._t0) * 1000.0 / max(1, self.hold_ms)); self.update()
        if self.progress >= 1.0: self._up(); self.held.emit()

    def paintEvent(self, e) -> None:
        super().paintEvent(e)
        if self.progress > 0:
            p = QPainter(self); p.setPen(Qt.PenStyle.NoPen); p.setBrush(_c(theme.ACC, 140))
            p.drawRoundedRect(QRectF(2, self.height() - 5, (self.width() - 4) * self.progress, 3), 1.5, 1.5); p.end()


class GateRow(QWidget):
    """One safety gate: a dot (filled = met, hollow = not met), the name, and its value in words."""
    def __init__(self, name: str) -> None:
        super().__init__()
        self.name = name; self.ok = False; self.value = "--"; self.setFixedHeight(24); self.setMinimumWidth(100)

    def set(self, ok: bool, value: str) -> None:
        if (ok, value) != (self.ok, self.value): self.ok, self.value = ok, value; self.update()

    def paintEvent(self, _e) -> None:
        p = QPainter(self); p.setRenderHint(QPainter.RenderHint.Antialiasing); c = QPointF(7, self.height() / 2)
        if self.ok: p.setPen(Qt.PenStyle.NoPen); p.setBrush(_c(theme.ACC))
        else: p.setPen(QPen(_c(theme.FAINT), 1.4)); p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawEllipse(c, 4.5, 4.5)
        p.setFont(theme.f_label(10)); p.setPen(_c(theme.TXT if self.ok else theme.DIM))
        p.drawText(QRectF(20, 0, self.width() - 20, self.height()), Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter, self.name)
        p.setFont(theme.f_mono(10)); p.setPen(_c(theme.DIM if self.ok else theme.AMB))
        p.drawText(QRectF(20, 0, self.width() - 22, self.height()), Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter, self.value)
        p.end()


class MotorButton(QPushButton):
    """Selection tile for one motor, laid out like the aircraft (front row on top): number, position, live output and a thin bar. Selected = accent border."""
    def __init__(self, n: int) -> None:
        super().__init__()
        self.n = n; self.label = _MOTOR[n][2]; self.us = 1000; self.stale = True; self.spinning = False
        self.setCheckable(True); self.setMinimumHeight(62); self.setCursor(Qt.CursorShape.PointingHandCursor)

    def nextCheckState(self) -> None:                          # selection is set by the tab, never toggled by a click
        pass

    def set_output(self, us: Optional[int]) -> None:
        self.stale = us is None
        if us is not None: self.us = int(us)
        self.update()

    def percent(self) -> float:
        return pct_of(self.us)

    def paintEvent(self, _e) -> None:
        p = QPainter(self); p.setRenderHint(QPainter.RenderHint.Antialiasing); r = QRectF(self.rect()).adjusted(1, 1, -1, -1); sel = self.isChecked()
        pen = QPen(_c(theme.ACC) if sel else _c(theme.HAIR2), 2 if sel else 1)
        if not self.isEnabled(): pen.setStyle(Qt.PenStyle.DashLine)
        p.setPen(pen); p.setBrush(_c(theme.HAIR if sel else theme.PANEL)); p.drawRoundedRect(r, 8, 8)
        f = theme.f_body(13); f.setBold(True); p.setFont(f); p.setPen(_c(theme.ACC if sel else theme.TXT if self.isEnabled() else theme.DISABLED))
        p.drawText(QRectF(r.x() + 12, r.y() + 6, 60, 20), Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter, f"M{self.n}")
        p.setFont(theme.f_label(9)); p.setPen(_c(theme.DIM)); p.drawText(QRectF(r.x() + 12, r.y() + 6, r.width() - 24, 20), Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter, self.label.upper())
        p.setFont(theme.f_mono(10)); p.setPen(_c(theme.ACC if self.spinning else theme.FAINT if self.stale else theme.DIM))
        txt = "SPINNING" if self.spinning else ("--" if self.stale else f"{self.us} µs  {self.percent():.0f} %")
        p.drawText(QRectF(r.x() + 12, r.y() + 27, r.width() - 24, 16), Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter, txt)
        bar = QRectF(r.x() + 12, r.bottom() - 11, r.width() - 24, 4); p.setPen(Qt.PenStyle.NoPen); p.setBrush(_c(theme.HAIR2)); p.drawRoundedRect(bar, 2, 2)
        if not self.stale and self.percent() > 0:
            p.setBrush(_c(theme.ACC)); p.drawRoundedRect(QRectF(bar.x(), bar.y(), max(4.0, bar.width() * min(1.0, self.percent() / MAX_PCT)), 4), 2, 2)
        p.end()


def _card(title: str) -> tuple:
    f = QFrame(); f.setObjectName("card"); v = QVBoxLayout(f); v.setContentsMargins(14, 12, 14, 12); v.setSpacing(10)
    h = QHBoxLayout(); t = QLabel(title); t.setFont(theme.f_label(11)); t.setStyleSheet(f"color:{theme.ACC};letter-spacing:2px"); h.addWidget(t); h.addStretch(1)
    meta = QLabel(""); meta.setFont(theme.f_label(10)); meta.setStyleSheet(f"color:{theme.DIM}"); h.addWidget(meta); v.addLayout(h)
    return f, v, meta


def _banner(text: str) -> QLabel:
    b = QLabel(text); b.setWordWrap(True); b.setFont(theme.f_body(12))
    b.setStyleSheet(f"color:{theme.AMB};border:1px solid {theme.AMB};border-radius:6px;padding:6px 10px;font-weight:700;background:transparent"); return b


# ---------------------------------------------------------------------------------------------------------------------------------------------------------- the tab
class MotorTab(QWidget):
    def __init__(self, win) -> None:
        super().__init__()
        self.win, self.model = win, win.model
        self.outputs: Optional[List[int]] = None; self.thr_us: Optional[int] = None; self.t_out = 0.0
        self._t_start: Optional[float] = None; self.unlocked_until = 0.0; self.selected = 1
        self.seq_on = False; self._seq_i = 0; self._seq_t = 0.0; self._seq_phase = "gap"
        root = QVBoxLayout(self); root.setContentsMargins(0, 0, 0, 0); root.setSpacing(12)
        head = QHBoxLayout(); t = QLabel("MOTOR TEST"); t.setFont(theme.f_label(12)); t.setStyleSheet(f"color:{theme.TXT};letter-spacing:3px")
        sub = QLabel("FC outputs and bench spin test, over the radio"); sub.setStyleSheet(f"color:{theme.DIM}"); sub.setFont(theme.f_body(12))
        self.chip = QLabel("ALL MOTORS IDLE"); self.chip.setFont(theme.f_label(10))
        head.addWidget(t); head.addSpacing(12); head.addWidget(sub); head.addStretch(1); head.addWidget(self.chip); root.addLayout(head)
        body = QHBoxLayout(); body.setSpacing(12); root.addLayout(body, 1)

        geo, gv, self.geo_meta = _card("FRAME GEOMETRY")                       # left: the aircraft
        self.view = FrameView(); self.view.motor_clicked.connect(self.select_motor); gv.addWidget(self.view, 1)
        hint = QLabel("Click a rotor to select it"); hint.setAlignment(Qt.AlignmentFlag.AlignHCenter); hint.setStyleSheet(f"color:{theme.FAINT}"); hint.setFont(theme.f_body(11)); gv.addWidget(hint)
        body.addWidget(geo, 5)

        outc, ov, self.out_meta = _card("FC OUTPUTS")                          # middle: the numbers
        bars = QHBoxLayout(); bars.setSpacing(2); self.bars = {}
        for n in (1, 2, 3, 4): self.bars[n] = ChannelBar(n); bars.addWidget(self.bars[n])
        ov.addLayout(bars, 1); body.addWidget(outc, 3)

        test, tv, self.test_meta = _card("BENCH MOTOR TEST")                   # right: the only controls that command anything
        gates = QGridLayout(); gates.setHorizontalSpacing(18); gates.setVerticalSpacing(2)
        self.g_link, self.g_disarmed, self.g_ground, self.g_unlock = GateRow("LINK"), GateRow("DISARMED"), GateRow("NOT IN FLIGHT"), GateRow("SAFETY UNLOCK")
        for i, g in enumerate((self.g_link, self.g_disarmed, self.g_ground, self.g_unlock)): gates.addWidget(g, i // 2, i % 2)
        tv.addLayout(gates)
        self.warn_spin = _banner("THIS SPINS THE MOTOR YOU HOLD: props on, keep hands and cables clear, hold or tether the drone (the unlock is you confirming this), release to stop.")
        tv.addWidget(self.warn_spin)
        self.btn_unlock = HoldPill("HOLD 1 s TO UNLOCK FOR 60 s"); self.btn_unlock.held.connect(self.unlock); tv.addWidget(self.btn_unlock)
        grid = QGridLayout(); grid.setHorizontalSpacing(8); grid.setVerticalSpacing(8); self.mbtn: List[MotorButton] = []
        place = {4: (0, 0), 2: (0, 1), 3: (1, 0), 1: (1, 1)}                    # like the aircraft from above, front row on top
        for n in (1, 2, 3, 4):
            b = MotorButton(n); b.clicked.connect(lambda _c=False, n=n: self.select_motor(n)); self.mbtn.append(b); grid.addWidget(b, *place[n])
        tv.addLayout(grid)
        thr = QHBoxLayout(); thr.setSpacing(8); tl = QLabel("THROTTLE"); tl.setFont(theme.f_label(10)); tl.setStyleSheet(f"color:{theme.DIM}"); thr.addWidget(tl)
        self.btn_minus = QPushButton("−"); self.btn_plus = QPushButton("+")
        for b in (self.btn_minus, self.btn_plus):
            b.setFixedSize(34, 30); b.setFont(theme.f_body(15)); b.setStyleSheet("padding:0px")
        self.sp = ThrottleSlider(); self.sp.changed.connect(self._on_throttle)
        self.btn_minus.clicked.connect(lambda: self.sp.setValue(self.sp.value() - STEP_PCT)); self.btn_plus.clicked.connect(lambda: self.sp.setValue(self.sp.value() + STEP_PCT))
        self.thr_val = QLabel(""); self.thr_val.setFont(theme.f_mono(11)); self.thr_val.setMinimumWidth(96); self.thr_val.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        thr.addWidget(self.btn_minus); thr.addWidget(self.sp, 1); thr.addWidget(self.btn_plus); thr.addWidget(self.thr_val); tv.addLayout(thr)
        self.btn_hold = QPushButton("HOLD TO SPIN"); self.btn_hold.setObjectName("primary"); self.btn_hold.setMinimumHeight(46)
        self.btn_hold.pressed.connect(self.on_hold_press); self.btn_hold.released.connect(self.on_release); tv.addWidget(self.btn_hold)
        row = QHBoxLayout(); row.setSpacing(8)
        self.btn_seq = QPushButton("SEQ 1 → 4"); self.btn_seq.setMinimumHeight(40); self.btn_seq.clicked.connect(self.toggle_sequence)
        self.btn_stop_all = QPushButton("STOP ALL"); self.btn_stop_all.setObjectName("danger"); self.btn_stop_all.setMinimumHeight(40); self.btn_stop_all.clicked.connect(self.stop_all)
        row.addWidget(self.btn_seq, 1); row.addWidget(self.btn_stop_all, 1); tv.addLayout(row)
        sep = QFrame(); sep.setFixedHeight(1); sep.setStyleSheet(f"background:{theme.HAIR}"); tv.addWidget(sep)
        al = QLabel("ALL FOUR MOTORS (ARMS THE FC)"); al.setFont(theme.f_label(10)); al.setStyleSheet(f"color:{theme.DIM};letter-spacing:1.5px"); tv.addWidget(al)
        self.warn_arm = _banner("THIS ARMS THE DRONE: all four motors spin at the test throttle (props on). Hold or tether the drone, keep hands and cables clear, someone on SAFE.")
        tv.addWidget(self.warn_arm)
        arow = QHBoxLayout(); arow.setSpacing(8)
        self.btn_arm = QPushButton("ARM + SPIN"); self.btn_arm.setMinimumHeight(40); self.btn_arm.clicked.connect(self.on_arm)
        self.btn_stop = QPushButton("STOP / DISARM"); self.btn_stop.setMinimumHeight(40); self.btn_stop.clicked.connect(self.on_stop)
        arow.addWidget(self.btn_arm, 1); arow.addWidget(self.btn_stop, 1); tv.addLayout(arow)
        self.note = QLabel(""); self.note.setWordWrap(True); self.note.setFont(theme.f_body(12)); tv.addWidget(self.note)
        self.status = QLabel("idle"); self.status.setWordWrap(True); self.status.setFont(theme.f_mono(11)); self.status.setStyleSheet(f"color:{theme.DIM}"); tv.addWidget(self.status)
        for b in (self.btn_seq, self.btn_stop_all, self.btn_arm, self.btn_stop, self.btn_hold, self.btn_unlock): b.setMinimumWidth(60)
        tv.addStretch(1); body.addWidget(test, 5)

        self.scale = QLabel(f"Scale 1000-2000 µs (the FC's motor output)  ·  dashed line = test ceiling {1000 + MAX_PCT * 10:.0f} µs ({MAX_PCT:g} %)  ·  "
                            f"a hover needs about 70 %  ·  each press stops by itself after 2.5 s; ARM + SPIN after {AUTO_STOP_S:.0f} s")
        self.scale.setStyleSheet(f"color:{theme.FAINT}"); self.scale.setFont(theme.f_body(11)); self.scale.setWordWrap(True); root.addWidget(self.scale)

        self._hold = QTimer(self); self._hold.setInterval(100); self._hold.timeout.connect(self._tick_hold)      # 10 Hz keep-alive while held / in the sequence
        self._ui = QTimer(self); self._ui.setInterval(200); self._ui.timeout.connect(self.refresh); self._ui.start()
        app = QApplication.instance()
        if app is not None: app.applicationStateChanged.connect(self._app_state); app.aboutToQuit.connect(self.stop_spin)
        win.model.core.bus.subscribe(T.TELEMETRY, self._on_tel)
        self.select_motor(1); self._on_throttle(self.sp.value()); self.refresh()

    # ---- data
    def _on_tel(self, msg) -> None:
        name, d = msg
        if name == "flight_mode" and "motors" in d:
            self.outputs, self.thr_us, self.t_out = list(d["motors"]), d.get("thr_us"), time.monotonic()

    @property
    def mt(self):
        return self.model.core.motor_test

    def running(self) -> bool:
        return self.model.core.ramp.phase in ("arming", "ramping")

    def unlocked(self) -> bool:
        return time.monotonic() < self.unlocked_until

    # ---- selection, throttle, unlock
    def select_motor(self, n: int) -> None:
        self.selected = int(n); self.view.set_selected(self.selected)
        for b in self.mbtn: b.setChecked(b.n == self.selected)

    def _on_throttle(self, v: float) -> None:
        us = int(round(1000 + 10 * v)); self.thr_val.setText(f"{v:.1f} % · {us} µs")
        try: self.mt.value_us = us                                              # the core clamps again (1100 us)
        except Exception: pass                                                  # noqa: BLE001
        if self.running(): self.model.core.ramp.cfg.start_thr = self.model.core.ramp.cfg.cap_thr = v / 100.0

    def unlock(self) -> None:
        self.unlocked_until = time.monotonic() + UNLOCK_S; self._note(f"Unlocked for {UNLOCK_S:.0f} s.", "ok"); self.refresh()

    # ---- hold-to-spin (one motor)
    def on_hold_press(self) -> None:
        if not self.unlocked():
            self._note("Locked: hold UNLOCK for 1 s first (drone held or tethered, hands clear).", "warning"); return
        self.start_spin(self.selected)

    def start_spin(self, n: int) -> bool:
        self.select_motor(n); why = self.mt.start(n)
        if why: self._note(why, "warning"); return False
        self._hold.start(); self.refresh(); return True

    def on_press(self, n: int) -> None:                                         # kept for callers: select, and spin when unlocked
        self.select_motor(n)
        if self.unlocked(): self.start_spin(n)

    def on_release(self) -> None:
        if not self.seq_on: self.stop_spin("release")

    def stop_spin(self, reason: str = "app exit") -> None:
        self._hold.stop(); self.seq_on = False; self.btn_seq.setText("SEQ 1 → 4")
        try: self.mt.stop(reason)
        except Exception: pass                                                  # noqa: BLE001
        self.refresh()

    def stop_all(self) -> None:
        self.stop_spin("STOP ALL")
        if self.running(): self.on_stop()
        self.unlocked_until = 0.0; self._note("Stopped. Unlock again to test.", "ok"); self.refresh()

    # ---- SEQ 1 -> 4: each motor SEQ_ON_S, a SEQ_GAP_S pause between; any stop / tab change / focus loss ends it
    def toggle_sequence(self) -> None:
        if self.seq_on: self.stop_spin("sequence stopped"); return
        if not self.unlocked(): self._note("Locked: hold UNLOCK for 1 s first (drone held or tethered, hands clear).", "warning"); return
        why = self.mt.refusal()
        if why: self._note(why, "warning"); return
        self.seq_on = True; self._seq_i = 0; self._seq_phase = "gap"; self._seq_t = time.monotonic() - SEQ_GAP_S
        self.btn_seq.setText("STOP SEQ"); self._hold.start(); self._tick_hold()

    def _tick_hold(self) -> None:
        now = time.monotonic()
        if self.seq_on:
            if self._seq_phase == "gap" and now - self._seq_t >= SEQ_GAP_S:
                if self._seq_i >= 4: self.stop_spin("sequence done"); self._note("Sequence done: M1, M2, M3, M4.", "ok"); return
                self._seq_i += 1
                if not self.start_spin(self._seq_i): self.stop_spin("sequence refused"); return
                self._seq_phase, self._seq_t = "on", now
            elif self._seq_phase == "on":
                if now - self._seq_t >= SEQ_ON_S: self.mt.stop("sequence step"); self._seq_phase, self._seq_t = "gap", now
                else: self.mt.refresh()
            self.refresh(); return
        self.mt.refresh()
        if self.mt.state != "spinning":
            self._hold.stop()
            if self.btn_hold.isDown(): self.btn_hold.setDown(False)
        self.refresh()

    def _app_state(self, st) -> None:
        if st != Qt.ApplicationState.ApplicationActive: self.stop_spin("window focus lost")

    def hideEvent(self, e) -> None:
        self.stop_spin("tab change"); super().hideEvent(e)

    # ---- ARM + SPIN (all four)
    def _block_reason(self) -> str:
        m = self.model
        if not m.connected: return "Connect to the drone first."
        if m.core.autonomy.phase != "idle" or self.running(): return "A flight or test is already running."
        return ""

    def on_arm(self) -> None:
        m = self.model; why = self._block_reason()
        if why: self._note(why, "warning"); return
        if m.source != "external": m.set_source("external")
        if not m.allow_arm: m.set_allow_arm(True)
        from nimbus_core.throttle_ramp import RampConfig
        v = min(self.sp.value(), MAX_PCT) / 100.0
        m.core.ramp.cfg = RampConfig(start_thr=v, cap_thr=v, rate_per_s=0.0, hold_at_cap_s=AUTO_STOP_S, lift_height_m=0.12, tilt_stop_deg=15.0)
        self.outputs = None
        if not m.core.ramp.start(): self._note("Cannot start: arming must be allowed and the controls source external.", "error"); return
        self._note("Motor test started.", "ok"); self._t_start = time.monotonic()

    def on_stop(self) -> None:
        self.model.core.ramp.stop("operator")

    # ---- display
    def _note(self, text: str, level: str = "ok") -> None:
        col = {"ok": theme.TXT, "warning": theme.AMB, "error": theme.RED}.get(level, theme.TXT)
        self.note.setText(text); self.note.setStyleSheet(f"color:{col}")

    def _chip(self, text: str, tone: str) -> None:
        col = {"idle": theme.DIM, "on": theme.ACC, "lock": theme.AMB}[tone]
        self.chip.setText(text); self.chip.setStyleSheet(f"color:{col};border:1px solid {col};border-radius:10px;padding:2px 10px;letter-spacing:1.5px")

    def refresh(self, *_a) -> None:
        now = time.monotonic(); mt = self.mt; run = self.running(); m = self.model
        fresh = self.outputs is not None and now - self.t_out < FRESH_S; outs = list(self.outputs) if fresh else None
        rb = getattr(mt, "readback", None)
        if getattr(mt, "state", "idle") != "idle" and rb is not None and now - getattr(mt, "t_readback", 0.0) < FRESH_S and (not fresh or mt.t_readback > self.t_out):
            outs = list(rb); fresh = True
        spinning = getattr(mt, "state", "idle") == "spinning"; spin_n = getattr(mt, "motor", 0) if spinning else 0
        self.view.set_outputs(outs, not fresh); self.view.set_spinning(spin_n)
        for n, b in self.bars.items(): b.set_output(outs[n - 1] if fresh and outs and len(outs) >= n else None, active=(n == spin_n))
        gate = mt.refusal() if hasattr(mt, "refusal") else ""
        lq = getattr(mt, "lq", None); armed = getattr(mt, "armed", None)
        self.g_link.set(bool(m.connected) and lq is not None and lq >= 50, "--" if lq is None else f"LQ {lq} %")
        self.g_disarmed.set(armed is False, "--" if armed is None else ("ARMED" if armed else "yes"))
        busy = run or m.core.autonomy.phase != "idle"
        self.g_ground.set(not busy, "flight / test running" if busy else "yes")
        left = self.unlocked_until - now; self.g_unlock.set(left > 0, f"{left:.0f} s" if left > 0 else "locked")
        for b in self.mbtn:
            b.set_output(outs[b.n - 1] if fresh and outs and len(outs) >= b.n else None); b.spinning = spin_n == b.n; b.update()
            b.setToolTip(f"Select M{b.n} ({b.label})" + (f"  ·  locked: {gate}" if gate else ""))
        self.btn_hold.setEnabled((not gate and left > 0) or self.btn_hold.isDown())
        self.btn_hold.setText(f"M{spin_n} SPINNING  ·  release to stop" if spinning and not self.seq_on else f"HOLD TO SPIN  M{self.selected}")
        self.btn_seq.setEnabled(self.seq_on or (not gate and left > 0))
        self.btn_arm.setEnabled(not self._block_reason()); self.btn_stop.setEnabled(run)
        if self.seq_on: self._chip(f"SEQUENCE  M{self._seq_i}", "on")
        elif spinning: self._chip(f"M{spin_n} SPINNING", "on")
        elif run: self._chip("ARMED · ALL MOTORS", "on")
        elif gate: self._chip("LOCKED", "lock")
        else: self._chip("ALL MOTORS IDLE", "idle")
        self.test_meta.setText("UNLOCKED" if left > 0 and not gate else "LOCKED")
        self.out_meta.setText("LIVE" if fresh else "NO DATA"); self.geo_meta.setText("QUAD X")
        st = getattr(mt, "status_text", "")
        if st and (getattr(mt, "state", "idle") != "idle" or st.startswith(("FC did not", "M"))) and not gate:
            self._note(st, "warning" if getattr(mt, "unconfirmed", False) else "ok")
        elif gate:                                                                  # a gate that is not met is always said, in words
            self._note("Motor buttons locked: " + gate, "warning")
        if run:
            left_r = max(0.0, AUTO_STOP_S - (now - (self._t_start or now)))
            self.status.setText(f"{m.core.ramp.phase.upper()}  throttle {m.core.ramp.thr * 100:.1f} %   (gyro calibration and arming take about 6 s first; stops itself in ~{left_r:.0f} s)")
        elif spinning:
            self.status.setText(f"M{spin_n} SPINNING (release to stop)")
        else:
            res = m.core.ramp.result
            self.status.setText("idle" if not res else f"stopped: {res.get('reason')}")
