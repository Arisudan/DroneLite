"""Drone3DView: a quad-X drone drawn with a plain QPainter perspective projection (no extra dependency).

World axes (metres): x right, y forward, z up. Left-drag orbits, wheel zooms, double-click or the right-click action resets the view.
Model: X frame with flat arms, motor bells, tri-blade 3-inch props (translucent blurred discs while they turn), slim stack, battery on top/back, camera at the front,
a FRONT arrow on the ground, ground grid and soft shadow. One light direction (LIGHT) shades every face; edges get a faint highlight.
Rotors turn in proportion to the live output. Grey = idle/stale, blue = running, red cross only for a motor the pilot marked NOT turning.
State reads without colour: stale -> the words NO DATA, running -> blurred discs and the output in us/percent, selected -> a thick ring and a filled label.
"""
from __future__ import annotations

import math
import time
from typing import List, Optional, Sequence

from PyQt6.QtCore import QPointF, QRectF, Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QAction, QColor, QFont, QPainter, QPen, QPolygonF
from PyQt6.QtWidgets import QSizePolicy, QWidget

from .. import theme

# Betaflight quad-X: motor number -> (right, forward) in units of the arm offset, spin sense (+1 = clockwise seen from above)
MOTOR_POS = {1: (+1.0, -1.0, -1), 2: (+1.0, +1.0, +1), 3: (-1.0, -1.0, +1), 4: (-1.0, +1.0, -1)}
ARM = 0.065            # m, motor offset along x and y
PROP_R = 0.038         # m, 3" prop
PROP_Z = 0.029         # m, prop plane height
DEF_YAW, DEF_PITCH, DEF_DIST = math.radians(35.0), math.radians(30.0), 0.50
IDLE_US = 1030
LIGHT = (-0.45, -0.35, 0.82)                       # world direction towards the light (upper left, behind the viewer's default side)
_n = math.sqrt(sum(c * c for c in LIGHT)); LIGHT = tuple(c / _n for c in LIGHT)
SPAN_M = (0.36, 0.25)                              # world width x height the default view frames (drone + labels)


def _rgb(c: str, a: int = 255) -> QColor:
    q = QColor(c); q.setAlpha(a); return q


def _shade(c: str, f: float, a: int = 255) -> QColor:
    q = QColor(c); return QColor(min(255, int(q.red() * f)), min(255, int(q.green() * f)), min(255, int(q.blue() * f)), a)


class Drone3DView(QWidget):
    motor_clicked = pyqtSignal(int)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setMinimumSize(360, 300); self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.setMouseTracking(False); self.setCursor(Qt.CursorShape.OpenHandCursor)
        self.outputs: List[int] = [1000] * 4
        self.stale = True
        self.selected = 0
        self.seen: List[Optional[bool]] = [None] * 4
        self.yaw, self.pitch, self.dist = DEF_YAW, DEF_PITCH, DEF_DIST
        self._ang = [0.0] * 4
        self._t = time.monotonic()
        self._press: Optional[QPointF] = None; self._last: Optional[QPointF] = None; self._moved = 0.0
        self._hit: dict = {}
        act = QAction("Reset view", self); act.triggered.connect(self.reset_view)
        self.addAction(act); self.setContextMenuPolicy(Qt.ContextMenuPolicy.ActionsContextMenu)
        self._timer = QTimer(self); self._timer.setInterval(33); self._timer.timeout.connect(self._tick); self._timer.start()

    # ---- API -----------------------------------------------------------------------------
    def set_outputs(self, outputs: Optional[Sequence[int]], stale: bool = False) -> None:
        if outputs is None:
            self.stale = True
        else:
            self.outputs = [int(o) for o in outputs]; self.stale = bool(stale)
        self.update()

    def set_selected(self, n: int) -> None:
        self.selected = int(n); self.update()

    def set_seen(self, seen: Sequence[Optional[bool]]) -> None:
        self.seen = list(seen); self.update()

    def reset_view(self) -> None:
        self.yaw, self.pitch, self.dist = DEF_YAW, DEF_PITCH, DEF_DIST; self.update()

    def running(self) -> bool:
        return (not self.stale) and max(self.outputs) > IDLE_US

    def _frac(self, i: int) -> float:
        """0..1 output of motor i (0 when stale/idle)."""
        return 0.0 if (self.stale or self.outputs[i] <= IDLE_US) else min(1.0, (self.outputs[i] - 1000) / 1000.0)

    # ---- animation ----------------------------------------------------------------------
    def _tick(self) -> None:
        now = time.monotonic(); dt = min(0.2, now - self._t); self._t = now
        if self.isVisible() and self.running():
            for i in range(4):
                if self._frac(i) > 0:
                    self._ang[i] = (self._ang[i] + (360.0 + 2400.0 * self._frac(i)) * dt) % 360.0       # visual only, proportional to the output
            self.update()

    # ---- projection ---------------------------------------------------------------------
    def _rot(self, p):
        x, y, z = p
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        x, y = x * cy - y * sy, x * sy + y * cy                                           # yaw about z
        cp, sp = math.cos(self.pitch), math.sin(self.pitch)
        y, z = y * cp - z * sp, y * sp + z * cp                                           # pitch about x: y becomes depth (away from viewer)
        return x, y, z

    def _proj(self, p):
        x, d, z = self._rot(p)
        depth = self.dist + d
        ppm = min(self.width() / SPAN_M[0], self.height() / SPAN_M[1]) * DEF_DIST          # pixels per metre at depth 1 (so the default view frames the drone)
        k = ppm / max(0.05, depth)
        return QPointF(self.width() / 2 + x * k, self.height() * 0.53 - z * k), depth

    def _poly(self, pts):
        return QPolygonF([self._proj(p)[0] for p in pts])

    # ---- primitives ---------------------------------------------------------------------
    def _obox(self, p: QPainter, c, ax, hs, base: str, edge_a: int = 70) -> None:
        """Oriented box: centre c, three orthonormal axes ax (world), half sizes hs. Visible faces only, light-shaded, faint edge highlight."""
        for k in range(3):
            for s in (-1, 1):
                nrm = ax[k]; nrm = (nrm[0] * s, nrm[1] * s, nrm[2] * s)
                if self._rot(nrm)[1] >= -0.02:
                    continue                                                              # facing away from the viewer
                o1, o2 = ax[(k + 1) % 3], ax[(k + 2) % 3]; h1, h2 = hs[(k + 1) % 3], hs[(k + 2) % 3]; ctr = tuple(c[j] + nrm[j] * hs[k] for j in range(3))
                pts = [tuple(ctr[j] + a * h1 * o1[j] + b * h2 * o2[j] for j in range(3)) for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
                lam = max(0.0, nrm[0] * LIGHT[0] + nrm[1] * LIGHT[1] + nrm[2] * LIGHT[2])
                p.setBrush(_shade(base, 0.55 + 0.75 * lam)); p.setPen(QPen(_shade(base, 1.9, edge_a), 1))
                p.drawPolygon(self._poly(pts))

    @staticmethod
    def _axes(a: float):
        ca, sa = math.cos(a), math.sin(a)
        return ((ca, sa, 0.0), (-sa, ca, 0.0), (0.0, 0.0, 1.0))

    def _ring(self, c, r, n=36, a0=0.0, a1=2 * math.pi):
        return [(c[0] + r * math.cos(a0 + (a1 - a0) * t / n), c[1] + r * math.sin(a0 + (a1 - a0) * t / n), c[2]) for t in range(n + 1)]

    # ---- painting -----------------------------------------------------------------------
    def paintEvent(self, _ev) -> None:
        p = QPainter(self); p.setRenderHint(QPainter.RenderHint.Antialiasing); p.setRenderHint(QPainter.RenderHint.TextAntialiasing)
        w, h = self.width(), self.height()
        p.fillRect(0, 0, w, h, _rgb(theme.PANEL))
        self._ground(p)
        items = []                                                                        # (depth, draw function), far first (painter's algorithm)
        for n, (mx, my, _s) in MOTOR_POS.items():
            items.append((self._rot((mx * ARM * 0.5, my * ARM * 0.5, 0.012))[1] + 1.0, lambda mx=mx, my=my: self._arm(p, mx, my)))     # arms sit below everything
        items.append((self._rot((0, 0, 0.02))[1], lambda: self._body(p)))
        for i, n in enumerate(MOTOR_POS):
            items.append((self._rot((MOTOR_POS[n][0] * ARM, MOTOR_POS[n][1] * ARM, 0.02))[1], lambda i=i, n=n: self._rotor(p, i, n)))
        for _d, fn in sorted(items, key=lambda t: -t[0]):
            fn()
        for i, n in enumerate(MOTOR_POS):
            self._label(p, i, n)
        p.setPen(_rgb(theme.DIM)); p.setFont(QFont("Lato", 9))
        p.drawText(QRectF(12, h - 24, w - 24, 16), Qt.AlignmentFlag.AlignLeft, "Drag to orbit, wheel to zoom, double-click to reset view")
        if self.stale:
            p.setPen(_rgb(theme.TXT)); p.setFont(QFont("Lato", 11, QFont.Weight.DemiBold))
            p.drawText(QRectF(0, 10, w, 22), Qt.AlignmentFlag.AlignCenter, "NO DATA  (motor outputs not received)")
        p.end()

    def _ground(self, p: QPainter) -> None:
        half, step, seg = 0.17, 0.034, 8
        p.setBrush(Qt.BrushStyle.NoBrush)
        for k in range(-5, 6):
            g = k * step
            for horiz in (0, 1):
                for s in range(seg):
                    t0, t1 = -half + 2 * half * s / seg, -half + 2 * half * (s + 1) / seg
                    a, b = ((t0, g, 0), (t1, g, 0)) if horiz else ((g, t0, 0), (g, t1, 0))
                    mid = math.hypot((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
                    al = max(0, int(120 * (1 - mid / (half * 1.25))))
                    p.setPen(QPen(_rgb(theme.HAIR2, al), 1)); p.drawLine(self._proj(a)[0], self._proj(b)[0])
        p.setPen(Qt.PenStyle.NoPen)                                                       # soft shadow: stacked translucent ellipses, offset away from the light
        for r, al in ((0.118, 14), (0.100, 18), (0.082, 24), (0.064, 30)):
            p.setBrush(_rgb("#000000", al))
            p.drawPolygon(self._poly(self._ring((0.012, 0.008, 0.0), r, 40)))
        # FRONT arrow flat on the ground, beyond the front props
        y0, y1, y2 = 0.118, 0.150, 0.176
        arrow = [(-0.005, y0, 0), (0.005, y0, 0), (0.005, y1, 0), (0.016, y1, 0), (0, y2, 0), (-0.016, y1, 0), (-0.005, y1, 0)]
        p.setBrush(_rgb(theme.TXT, 215)); p.setPen(QPen(_rgb(theme.TXT, 255), 1)); p.drawPolygon(self._poly(arrow))
        tip = self._proj((0, y2 + 0.012, 0))[0]
        p.setPen(_rgb(theme.TXT)); p.setFont(QFont("Lato", 9, QFont.Weight.Bold))
        p.drawText(QRectF(tip.x() - 40, tip.y() - 17, 80, 16), Qt.AlignmentFlag.AlignCenter, "FRONT")

    def _arm(self, p: QPainter, mx: float, my: float) -> None:
        a = math.atan2(my, mx); L = math.hypot(mx, my) * ARM
        self._obox(p, (mx * ARM / 2, my * ARM / 2, 0.0125), self._axes(a), (L / 2 + 0.004, 0.0055, 0.0022), "#4b535f")

    def _body(self, p: QPainter) -> None:
        ident = self._axes(0.0)
        self._obox(p, (0, 0, 0.0115), ident, (0.0235, 0.043, 0.0022), "#3a414b")                    # bottom plate
        self._obox(p, (0, 0.002, 0.0172), ident, (0.0165, 0.030, 0.0042), "#262b33")                # electronics stack
        self._obox(p, (0, 0, 0.0232), ident, (0.0225, 0.040, 0.0022), "#454d59")                    # top plate
        self._obox(p, (0, -0.012, 0.0325), ident, (0.0150, 0.0300, 0.0071), "#323a46", 90)           # battery on top, towards the back
        self._obox(p, (0, -0.012, 0.0325), ident, (0.0156, 0.0050, 0.0074), "#1c2027", 40)           # battery strap
        self._obox(p, (0, 0.046, 0.0235), ident, (0.0115, 0.0085, 0.0085), "#1c2027", 90)           # camera at the front
        lens = self._proj((0, 0.0545, 0.0235))[0]; r = abs(self._proj((0.0055, 0.0545, 0.0235))[0].x() - lens.x()) + 1
        if self._rot((0, 1, 0))[1] < 0:                                                           # front face toward the viewer
            p.setBrush(_rgb("#0b1626")); p.setPen(QPen(_rgb(theme.ACC, 200), 1.2)); p.drawEllipse(lens, r, r * max(0.5, math.cos(self.pitch)))

    def _rotor(self, p: QPainter, i: int, n: int) -> None:
        mx, my, sense = MOTOR_POS[n]
        c = (mx * ARM, my * ARM, 0.0); ctr = self._proj((c[0], c[1], PROP_Z))[0]
        frac = self._frac(i); run = frac > 0
        col = theme.RED if self.seen[i] is False else (theme.ACC if run else theme.DIM)
        sel = (n == self.selected)
        # motor bell (cylinder, 16 sides) and its top cap
        zb0, zb1 = 0.0105, 0.0245
        sides = []
        for t in range(16):
            a0, a1 = t * math.pi / 8, (t + 1) * math.pi / 8; am = (a0 + a1) / 2
            nrm = (math.cos(am), math.sin(am), 0.0)
            if self._rot(nrm)[1] >= 0.05:
                continue
            lam = max(0.0, nrm[0] * LIGHT[0] + nrm[1] * LIGHT[1]) + 0.25
            q = [(c[0] + 0.0085 * math.cos(a), c[1] + 0.0085 * math.sin(a), z) for a, z in ((a0, zb0), (a1, zb0), (a1, zb1), (a0, zb1))]
            sides.append((q, lam))
        for q, lam in sides:
            p.setBrush(_shade("#59616d", 0.5 + 0.8 * lam)); p.setPen(Qt.PenStyle.NoPen); p.drawPolygon(self._poly(q))
        p.setBrush(_shade("#79828f", 1.0)); p.setPen(QPen(_rgb("#a2abb8", 110), 1)); p.drawPolygon(self._poly(self._ring((c[0], c[1], zb1), 0.0085, 20)))
        # prop: translucent blurred disc (stronger the faster it turns) + three blades
        pc = (c[0], c[1], PROP_Z); p.setPen(Qt.PenStyle.NoPen)
        if run:
            for rr, al in ((1.0, 22), (0.88, 26), (0.74, 30), (0.5, 22)):
                p.setBrush(_rgb(col, int(al * (0.6 + 0.6 * frac)))); p.drawPolygon(self._poly(self._ring(pc, PROP_R * rr, 36)))
        else:
            p.setBrush(_rgb(col, 18)); p.drawPolygon(self._poly(self._ring(pc, PROP_R, 36)))
        p.setPen(QPen(_rgb(col, 150 if run else 90), 1)); p.setBrush(Qt.BrushStyle.NoBrush); p.drawPolygon(self._poly(self._ring(pc, PROP_R, 36)))
        shape = [(0.006, -0.003), (0.014, -0.0065), (0.028, -0.0078), (PROP_R, -0.0045), (PROP_R, 0.0025), (0.028, 0.0040), (0.012, 0.0030), (0.006, 0.0035)]
        trail = (0, 9, 18, 27) if run else (0,)
        for ti, dt in enumerate(trail):
            al = (200 if not run else (120 - ti * 28))
            p.setBrush(_rgb("#cfd5de" if not run else col, max(20, al))); p.setPen(Qt.PenStyle.NoPen if run else QPen(_rgb("#ffffff", 60), 1))
            for b in range(3):
                a = math.radians(self._ang[i] * -sense + b * 120 + dt * sense)
                ca, sa = math.cos(a), math.sin(a)
                p.drawPolygon(self._poly([(pc[0] + u * ca - v * sa, pc[1] + u * sa + v * ca, PROP_Z) for u, v in shape]))
        p.setBrush(_rgb("#2b3038")); p.setPen(QPen(_rgb("#8e97a4", 160), 1)); p.drawPolygon(self._poly(self._ring((c[0], c[1], PROP_Z + 0.003), 0.0055, 16)))   # hub nut
        # spin arrow: arc on the OUTER side of the prop (away from the body), head shows the sense (clockwise = +1, seen from above)
        out = math.atan2(my, mx); span = math.radians(110); ra = PROP_R * 1.16
        a0 = out + sense * span / 2; a1 = out - sense * span / 2                           # clockwise seen from above = decreasing angle
        pts = [self._proj(q)[0] for q in self._ring(pc, ra, 12, a0, a1)]
        p.setPen(QPen(_rgb(theme.TXT if sel else theme.DIM, 230), 1.6)); p.setBrush(Qt.BrushStyle.NoBrush)
        for q0, q1 in zip(pts, pts[1:]):
            p.drawLine(q0, q1)
        tip, prev = pts[-1], pts[-3]; ang = math.atan2(tip.y() - prev.y(), tip.x() - prev.x())
        p.setBrush(_rgb(theme.TXT if sel else theme.DIM)); p.setPen(Qt.PenStyle.NoPen)
        p.drawPolygon(QPolygonF([tip, QPointF(tip.x() - 8 * math.cos(ang - 0.45), tip.y() - 8 * math.sin(ang - 0.45)), QPointF(tip.x() - 8 * math.cos(ang + 0.45), tip.y() - 8 * math.sin(ang + 0.45))]))
        if sel:                                                                           # selected: thick ring (plus a filled label, see _label)
            p.setBrush(Qt.BrushStyle.NoBrush); p.setPen(QPen(_rgb(theme.ACC), 3)); p.drawPolygon(self._poly(self._ring(pc, PROP_R * 1.3, 48)))
        if self.seen[i] is False:
            p.setPen(QPen(_rgb(theme.RED), 3)); p.drawLine(QPointF(ctr.x() - 12, ctr.y() - 12), QPointF(ctr.x() + 12, ctr.y() + 12)); p.drawLine(QPointF(ctr.x() - 12, ctr.y() + 12), QPointF(ctr.x() + 12, ctr.y() - 12))
        rr = max(abs(self._proj((c[0] + PROP_R, c[1], PROP_Z))[0].x() - ctr.x()), abs(self._proj((c[0], c[1] + PROP_R, PROP_Z))[0].y() - ctr.y()))
        self._hit[n] = (ctr, rr * 1.25 + 4)

    def _label(self, p: QPainter, i: int, n: int) -> None:
        """M-number, spin sense and live output in a box beyond the prop on the outward side: it never touches the body or the props."""
        mx, my, sense = MOTOR_POS[n]
        ctr, _r = self._hit.get(n, (None, 0))
        if ctr is None:
            return
        body = self._proj((0, 0, PROP_Z))[0]
        # outermost screen extent of the prop + spin arc, measured along the outward direction
        far = self._proj((mx * (ARM + PROP_R * 0.9), my * (ARM + PROP_R * 0.9), PROP_Z))[0]
        vx, vy = far.x() - body.x(), far.y() - body.y(); vn = max(1.0, math.hypot(vx, vy)); vx, vy = vx / vn, vy / vn
        bw, bh = 96.0, 34.0
        ext = abs(vx) * bw / 2 + abs(vy) * bh / 2 + 12
        cx, cy = far.x() + vx * ext, far.y() + vy * ext
        cx = min(max(cx, bw / 2 + 4), self.width() - bw / 2 - 4); cy = min(max(cy, bh / 2 + 30), self.height() - bh / 2 - 28)
        rect = QRectF(cx - bw / 2, cy - bh / 2, bw, bh); sel = (n == self.selected)
        p.setPen(QPen(_rgb(theme.ACC if sel else theme.HAIR2), 1.5 if sel else 1)); p.setBrush(_rgb(theme.HAIR if sel else theme.BG, 235))
        p.drawRoundedRect(rect, 4, 4)
        p.setPen(_rgb(theme.ACC if sel else theme.TXT)); p.setFont(QFont("Lato", 10, QFont.Weight.Bold))
        p.drawText(QRectF(rect.x() + 6, rect.y() + 2, bw - 12, 16), Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter, f"M{n}")
        p.setPen(_rgb(theme.DIM)); p.drawText(QRectF(rect.x() + 6, rect.y() + 2, bw - 12, 16), Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter, "CW" if sense > 0 else "CCW")
        p.setPen(_rgb(theme.DIM)); p.setFont(QFont("Noto Sans Mono", 8))
        pct = max(0.0, (self.outputs[i] - 1000) / 10.0)
        p.drawText(QRectF(rect.x() + 6, rect.y() + 17, bw - 12, 14), Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter, "--" if self.stale else f"{self.outputs[i]} us  {pct:.0f} %")

    # ---- mouse --------------------------------------------------------------------------
    def mousePressEvent(self, e) -> None:
        if e.button() == Qt.MouseButton.LeftButton:
            self._press = self._last = e.position(); self._moved = 0.0; self.setCursor(Qt.CursorShape.ClosedHandCursor)

    def mouseMoveEvent(self, e) -> None:
        if self._last is not None:
            d = e.position() - self._last; self._last = e.position(); self._moved += abs(d.x()) + abs(d.y())
            self.yaw -= d.x() * 0.01; self.pitch = min(1.45, max(0.08, self.pitch + d.y() * 0.01)); self.update()

    def mouseReleaseEvent(self, e) -> None:
        if e.button() == Qt.MouseButton.LeftButton and self._press is not None:
            if self._moved < 4.0:
                self.click_at(e.position())
            self._press = self._last = None; self.setCursor(Qt.CursorShape.OpenHandCursor)

    def mouseDoubleClickEvent(self, e) -> None:
        self.reset_view()

    def wheelEvent(self, e) -> None:
        self.dist = min(1.4, max(0.3, self.dist * (0.9 if e.angleDelta().y() > 0 else 1.1))); self.update()

    def click_at(self, pos: QPointF) -> None:
        best, bd = 0, 1e9
        for n, (c, r) in self._hit.items():
            d = math.hypot(pos.x() - c.x(), pos.y() - c.y())
            if d <= r and d < bd:
                best, bd = n, d
        if best:
            self.motor_clicked.emit(best)
