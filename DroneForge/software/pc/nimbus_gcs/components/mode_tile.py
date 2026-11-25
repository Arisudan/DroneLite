"""ModeTile: a 3D 'keycap' button for the FLIGHT MODE card (2026-11-24, owner: 3D, modern, with an AI-style light trace).

Raised tile = a darker lip under the face (the depth), a soft drop shadow, a top highlight and a gentle vertical gradient. The ACTIVE mode sits pressed in, filled with
the accent, with an outer glow and a thin light trace that runs around its edge (a conical-gradient band, 1.6 s per lap, painted only while it is active and visible).
Hover lifts the face 1 px, a press pushes it down. Disabled = flat, dashed outline, nothing animated. Colours only from theme.py (greys + the one accent).
A QAbstractButton, so text() / setText() / isEnabled() / click() behave like any button (the page and its tests use them); a leading '● ' in the text marks the active
mode for those callers and is not painted (the tile shows it by its state)."""
from __future__ import annotations

import math
import time

from PyQt6.QtCore import QPointF, QRectF, QSize, Qt, QTimer
from PyQt6.QtGui import QColor, QConicalGradient, QLinearGradient, QPainter, QPainterPath, QPen
from PyQt6.QtWidgets import QAbstractButton, QSizePolicy

from .. import theme

DEPTH, RAD = 4.0, 10.0
TRACE_LAP_S = 1.6


def _c(tok: str, a: int = 255) -> QColor:
    c = QColor(tok); c.setAlpha(a); return c


class ModeTile(QAbstractButton):
    def __init__(self, text: str, sub: str = "", icon: str = "", parent=None) -> None:
        super().__init__(parent)
        self.setText(text); self.sub = sub; self.icon = icon; self.active = False; self.pending = False; self.compact = False; self._hover = False
        self.setCursor(Qt.CursorShape.PointingHandCursor); self.setMouseTracking(True)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed); self.setFixedHeight(64)
        self._t0 = time.monotonic(); self._anim = QTimer(self); self._anim.setInterval(33); self._anim.timeout.connect(self.update)

    # ---- API
    def set_active(self, on: bool) -> None:
        if on != self.active:
            self.active = on; self._sync_anim(); self.update()

    def set_pending(self, on: bool) -> None:
        """Pressed and sent, the FC has not confirmed yet: an accent outline, no fill (cleared when the FC reports)."""
        if on != self.pending: self.pending = on; self.update()

    def set_sub(self, s: str) -> None:
        if s != self.sub: self.sub = s; self.update()

    def set_compact(self, on: bool) -> None:
        self.compact = on; self.update()

    def label(self) -> str:
        return self.text().replace("● ", "").replace("●", "").strip()

    def sizeHint(self) -> QSize:
        return QSize(110, self.height())

    # ---- animation only while it shows something
    def _sync_anim(self) -> None:
        if not hasattr(self, "_anim"): return                                       # Qt sends change events while __init__ runs
        run = self.active and self.isEnabled() and self.isVisible()
        if run and not self._anim.isActive(): self._anim.start()
        elif not run and self._anim.isActive(): self._anim.stop()

    def showEvent(self, e) -> None:
        super().showEvent(e); self._sync_anim()

    def hideEvent(self, e) -> None:
        super().hideEvent(e); self._sync_anim()

    def changeEvent(self, e) -> None:
        super().changeEvent(e); self._sync_anim()

    def enterEvent(self, e) -> None:
        self._hover = True; self.update(); super().enterEvent(e)

    def leaveEvent(self, e) -> None:
        self._hover = False; self.update(); super().leaveEvent(e)

    # ---- paint
    def paintEvent(self, _e) -> None:
        p = QPainter(self); p.setRenderHint(QPainter.RenderHint.Antialiasing)
        W, H = self.width(), self.height(); en = self.isEnabled(); act = self.active and en
        down = self.isDown() and en
        depth = DEPTH if en else 0.0                                                 # the same geometry in EVERY state: nothing moves when a tile is pressed, connected or active
        top = 1.0
        face = QRectF(2, top, W - 4, H - DEPTH - 3)
        if down: face.translate(0, 1.0)                                              # a press: 1 px only, text moves with it
        if en:
            sh = QRectF(face.x() + 1, face.y() + depth + 2, face.width() - 2, face.height())       # soft drop shadow
            for k, a in ((3, 26), (2, 34), (1, 46)):
                p.setPen(Qt.PenStyle.NoPen); p.setBrush(_c(theme.BG, a * 2)); p.drawRoundedRect(sh.adjusted(-k, -k + 1, k, k), RAD + k, RAD + k)
            if depth > 0:                                                            # the lip: the tile's side, seen below the face
                lip = QRectF(face.x(), face.y() + depth, face.width(), face.height())
                p.setBrush(_c(theme.ACC_DEEP, 255) if act else _c(theme.BG)); p.drawRoundedRect(lip, RAD, RAD)
        if act:                                                                      # outer glow
            for k, a in ((6, 22), (4, 34), (2, 52)):
                p.setPen(QPen(_c(theme.ACC, a), 2)); p.setBrush(Qt.BrushStyle.NoBrush); p.drawRoundedRect(face.adjusted(-k / 2, -k / 2, k / 2, k / 2), RAD + k / 2, RAD + k / 2)
        g = QLinearGradient(face.topLeft(), face.bottomLeft())
        if act: g.setColorAt(0, _c(theme.ACC)); g.setColorAt(1, _c(theme.ACC_DEEP))
        elif en:
            g.setColorAt(0, _c(theme.HAIR2 if (self._hover or down) else theme.HAIR)); g.setColorAt(1, _c(theme.PANEL))
        else: g.setColorAt(0, _c(theme.PANEL)); g.setColorAt(1, _c(theme.PANEL))
        pen = QPen(_c(theme.HAIR2), 1)
        if not en: pen.setStyle(Qt.PenStyle.DashLine)
        elif act: pen = QPen(_c(theme.ACC), 1)
        elif self.pending: pen = QPen(_c(theme.ACC), 2)
        p.setPen(pen); p.setBrush(g); p.drawRoundedRect(face, RAD, RAD)
        if en:                                                                       # top highlight
            hl = QPainterPath(); hl.addRoundedRect(face.adjusted(1.5, 1.5, -1.5, -face.height() * 0.55), RAD - 1.5, RAD - 1.5)
            hg = QLinearGradient(face.topLeft(), QPointF(face.x(), face.y() + face.height() * 0.45))
            hg.setColorAt(0, _c(theme.TXT, 70 if act else 34)); hg.setColorAt(1, _c(theme.TXT, 0))
            p.setPen(Qt.PenStyle.NoPen); p.setBrush(hg); p.drawPath(hl)
        if act:                                                                      # the AI trace: a light band running around the edge
            ang = (360.0 * ((time.monotonic() - self._t0) / TRACE_LAP_S)) % 360.0
            cg = QConicalGradient(face.center(), -ang)
            cg.setColorAt(0.00, _c(theme.TXT, 230)); cg.setColorAt(0.06, _c(theme.ACC, 160)); cg.setColorAt(0.16, _c(theme.ACC, 0))
            cg.setColorAt(0.84, _c(theme.ACC, 0)); cg.setColorAt(0.94, _c(theme.ACC, 120)); cg.setColorAt(1.00, _c(theme.TXT, 230))
            p.setPen(QPen(cg, 2.0)); p.setBrush(Qt.BrushStyle.NoBrush); p.drawRoundedRect(face.adjusted(1, 1, -1, -1), RAD - 1, RAD - 1)
        fg = theme.ON_ACC if act else (theme.TXT if en else theme.DISABLED); sub_c = _c(theme.ON_ACC, 170) if act else _c(theme.DIM if en else theme.DISABLED)
        mid = face.center().x()
        if self.compact:                                                            # small window: the label only, centred
            f = theme.f_label(10); f.setBold(True); p.setFont(f); p.setPen(_c(fg))
            p.drawText(face, Qt.AlignmentFlag.AlignCenter, self.label())
        else:                                                                        # icon on top, label, then what it does
            ic = 16.0; gy = face.y() + 6 + ic / 2
            if self.icon: self._glyph(p, self.icon, QPointF(mid, gy), ic, _c(fg))
            f = theme.f_label(10); f.setBold(True); p.setFont(f); p.setPen(_c(fg))
            p.drawText(QRectF(face.x() + 2, gy + ic / 2 + 2, face.width() - 4, 16), Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignVCenter, self.label())
            if self.sub:
                p.setFont(theme.f_body(9)); p.setPen(sub_c)
                p.drawText(QRectF(face.x() + 2, gy + ic / 2 + 17, face.width() - 4, 14), Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignVCenter, self.sub)
        p.end()

    def _glyph(self, p: QPainter, kind: str, c: QPointF, s: float, col: QColor) -> None:
        p.save(); pen = QPen(col, 1.6); pen.setCapStyle(Qt.PenCapStyle.RoundCap); p.setPen(pen); p.setBrush(Qt.BrushStyle.NoBrush); h = s / 2
        if kind == "alt":                                                            # height: up/down arrows on a level bar
            p.drawLine(QPointF(c.x(), c.y() - h), QPointF(c.x(), c.y() + h))
            for d in (-1, 1):
                p.drawLine(QPointF(c.x(), c.y() + d * h), QPointF(c.x() - h * 0.45, c.y() + d * h * 0.55)); p.drawLine(QPointF(c.x(), c.y() + d * h), QPointF(c.x() + h * 0.45, c.y() + d * h * 0.55))
            p.drawLine(QPointF(c.x() - h, c.y()), QPointF(c.x() + h, c.y()))
        elif kind == "pos":                                                          # position: crosshair on a ring
            p.drawEllipse(c, h * 0.7, h * 0.7)
            for a in (0, 90, 180, 270):
                r = math.radians(a); p.drawLine(QPointF(c.x() + math.cos(r) * h * 0.45, c.y() + math.sin(r) * h * 0.45), QPointF(c.x() + math.cos(r) * h, c.y() + math.sin(r) * h))
            p.setBrush(col); p.drawEllipse(c, 1.8, 1.8)
        elif kind == "explore":                                                      # explore: a path with a scan arc
            path = QPainterPath(QPointF(c.x() - h, c.y() + h * 0.7)); path.cubicTo(QPointF(c.x() - h * 0.2, c.y() + h * 0.7), QPointF(c.x() - h * 0.4, c.y() - h * 0.2), QPointF(c.x() + h * 0.3, c.y() - h * 0.3))
            p.drawPath(path); p.drawArc(QRectF(c.x() - h * 0.1, c.y() - h, h * 1.1, h * 1.1), 300 * 16, 120 * 16)
            p.setBrush(col); p.drawEllipse(QPointF(c.x() + h * 0.3, c.y() - h * 0.3), 2.0, 2.0)
        p.restore()
