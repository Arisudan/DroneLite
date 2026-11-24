"""Battery chip for the top bar (2026-11-24 redesign): the same 32 px rounded hairline chip as LINK / MODE, holding a phone-style battery: a rounded body with
FIVE cells (filled by the charge, the last one partly), a terminal nub, a soft inner gloss, then the percentage in tabular figures and the pack voltage small beside it.
Accent cells normally, amber cells + amber chip edge + a caution triangle below 30 % (a WARNING only: nothing is blocked or landed on the battery);
with no reading: empty dashed cells and '--'. Colours only from theme.py."""
from __future__ import annotations

from typing import Optional

from PyQt6.QtCore import QPointF, QRectF, QSize, Qt
from PyQt6.QtGui import QColor, QFontMetricsF, QLinearGradient, QPainter, QPainterPath, QPen
from PyQt6.QtWidgets import QSizePolicy, QWidget

from .. import theme
from . import icons

LOW_PCT = 30.0
CELLS = 5


def _c(tok: str, a: int = 255) -> QColor:
    c = QColor(tok); c.setAlpha(a); return c


class BatteryGauge(QWidget):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.pct: Optional[float] = None; self.volts: Optional[float] = None; self._level = "normal"
        self.setFixedHeight(32); self.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
        self.setToolTip("Battery: charge and pack voltage (the FC reading with your voltage correction). Below 30 % it turns amber: a warning only.")

    # same small API as StatusChip (tests, callers)
    def level(self) -> str:
        return self._level

    def value(self) -> str:
        return "--" if self.pct is None else f"{self.pct:.0f}%"

    def set_battery(self, pct: Optional[float], volts: Optional[float]) -> None:
        self.pct = None if pct is None else max(0.0, min(100.0, float(pct))); self.volts = volts
        self._level = "normal" if (self.pct is None or self.pct >= LOW_PCT) else "caution"
        self.updateGeometry(); self.update()

    def _vtxt(self) -> str:
        return "" if self.volts is None else f"{self.volts:.1f} V"

    def sizeHint(self) -> QSize:
        w = 12 + 34 + 10 + QFontMetricsF(theme.f_mono(13)).horizontalAdvance("100%") + 8 + QFontMetricsF(theme.f_mono(10)).horizontalAdvance("16.8 V") + 12
        if self._level != "normal": w += 22
        return QSize(int(w), 32)

    def minimumSizeHint(self) -> QSize:
        return self.sizeHint()

    def paintEvent(self, _e) -> None:
        p = QPainter(self); p.setRenderHint(QPainter.RenderHint.Antialiasing); p.setRenderHint(QPainter.RenderHint.TextAntialiasing)
        r = QRectF(self.rect()); cy = r.center().y(); none = self.pct is None; caut = self._level == "caution"
        edge = theme.AMB if caut else theme.HAIR2
        chip = QPainterPath(); chip.addRoundedRect(r.adjusted(.5, .5, -.5, -.5), 6, 6)
        p.setPen(QPen(_c(edge), 1)); p.setBrush(Qt.BrushStyle.NoBrush); p.drawPath(chip)
        # the battery: body, nub, five cells
        body = QRectF(12, cy - 7.5, 30, 15); nub = QRectF(body.right() + 1.2, cy - 3.5, 2.6, 7)
        outline = theme.AMB if caut else (theme.FAINT if none else theme.DIM)
        pen = QPen(_c(outline), 1.3)
        if none: pen.setStyle(Qt.PenStyle.DashLine)
        p.setPen(pen); p.setBrush(_c(theme.BG)); p.drawRoundedRect(body, 3.5, 3.5)
        p.setPen(Qt.PenStyle.NoPen); p.setBrush(_c(outline)); p.drawRoundedRect(nub, 1.2, 1.2)
        inner = body.adjusted(2.5, 2.5, -2.5, -2.5); gap = 1.4; cw = (inner.width() - gap * (CELLS - 1)) / CELLS
        fill = 0.0 if none else self.pct / 100.0 * CELLS
        for i in range(CELLS):
            cell = QRectF(inner.x() + i * (cw + gap), inner.y(), cw, inner.height())
            p.setBrush(_c(theme.HAIR, 160)); p.drawRoundedRect(cell, 1.2, 1.2)                   # the empty cell
            part = max(0.0, min(1.0, fill - i))
            if part > 0:
                g = QLinearGradient(cell.topLeft(), cell.bottomLeft()); base = theme.AMB if caut else theme.ACC
                g.setColorAt(0, _c(theme.TXT, 230) if not caut else _c(base)); g.setColorAt(0.35, _c(base)); g.setColorAt(1, _c(theme.ACC_DEEP if not caut else base, 230))
                p.setBrush(g); p.drawRoundedRect(QRectF(cell.x(), cell.y(), cell.width() * part, cell.height()), 1.2, 1.2)
        gl = QLinearGradient(body.topLeft(), QPointF(body.x(), body.center().y()))                # gloss
        gl.setColorAt(0, _c(theme.TXT, 40)); gl.setColorAt(1, _c(theme.TXT, 0))
        p.setBrush(gl); p.drawRoundedRect(body.adjusted(1, 1, -1, -body.height() / 2), 2.5, 2.5)
        # the numbers
        x = nub.right() + 10
        f = theme.f_mono(13); f.setBold(True); p.setFont(f); p.setPen(_c(theme.AMB if caut else (theme.DIM if none else theme.TXT)))
        w = QFontMetricsF(f).horizontalAdvance(self.value())
        p.drawText(QRectF(x, 0, w + 2, r.height()), Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignLeft, self.value()); x += w + 8
        if self._vtxt():
            vf = theme.f_mono(10); p.setFont(vf); p.setPen(_c(theme.FAINT))
            vw = QFontMetricsF(vf).horizontalAdvance(self._vtxt())
            p.drawText(QRectF(x, 1, vw + 2, r.height()), Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignLeft, self._vtxt()); x += vw + 6
        if caut:
            icons.draw_icon("caution", p, QPointF(x + 9, cy), _c(theme.AMB), 16)
        p.end()
