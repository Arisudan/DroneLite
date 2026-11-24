"""Top-bar chip: icon + SMALL LABEL + value. Levels: normal | caution (amber + triangle) | alarm (red). Text always carries the meaning."""
from __future__ import annotations

from PyQt6.QtCore import QPointF, QRectF, QSize, Qt
from PyQt6.QtGui import QFontMetricsF, QPainter, QPainterPath, QPen

from PyQt6.QtWidgets import QWidget

from .. import theme
from . import icons

LEVELS = ("normal", "caution", "alarm")


class StatusChip(QWidget):
    def __init__(self, label="", value="", icon="", level="normal", parent=None):
        super().__init__(parent)
        self._label, self._value, self._icon, self._level = label, value, icon, level
        self.setFixedHeight(32)

    def level(self) -> str:
        return self._level

    def value(self) -> str:
        return self._value

    def label(self) -> str:
        return self._label

    def set_value(self, text: str, level: str = "normal"):
        self._value, self._level = text, (level if level in LEVELS else "normal")
        self.updateGeometry(); self.update()

    def _colors(self):
        if self._level == "caution":
            return theme.AMB, theme.AMB, theme.AMB
        if self._level == "alarm":
            return theme.RED, theme.RED, theme.RED
        return theme.HAIR2, theme.DIM, theme.TXT

    def sizeHint(self):
        w = 12 + 12
        if self._icon:
            w += 22
        if self._level != "normal":
            w += 22
        if self._label:
            w += QFontMetricsF(theme.f_label(10)).horizontalAdvance(self._label.upper()) + 10
        w += QFontMetricsF(theme.f_mono(13)).horizontalAdvance(self._value)
        return QSize(int(w + 1), 32)

    def minimumSizeHint(self):
        return self.sizeHint()

    def paintEvent(self, e):
        p = QPainter(self); p.setRenderHint(QPainter.RenderHint.Antialiasing); p.setRenderHint(QPainter.RenderHint.TextAntialiasing)
        edge, lab, val = self._colors(); r = QRectF(self.rect()); cy = r.center().y()
        path = QPainterPath(); path.addRoundedRect(r.adjusted(.5, .5, -.5, -.5), 6, 6)
        p.setPen(QPen(theme.qcolor(edge), 1)); p.setBrush(Qt.BrushStyle.NoBrush); p.drawPath(path)
        x = 12.0
        if self._icon:
            icons.draw_icon(self._icon, p, QPointF(x + 7, cy), theme.qcolor(lab if self._level == "normal" else val), 16); x += 22
        if self._label:
            p.setFont(theme.f_label(10)); p.setPen(theme.qcolor(lab)); w = QFontMetricsF(p.font()).horizontalAdvance(self._label.upper())
            p.drawText(QRectF(x, 0, w + 2, r.height()), Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignLeft, self._label.upper()); x += w + 10
        p.setFont(theme.f_mono(13)); p.setPen(theme.qcolor(val))
        p.drawText(QRectF(x, 0, r.width() - x, r.height()), Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignLeft, self._value)
        if self._level != "normal":      # shape cue so the state reads without colour
            w = QFontMetricsF(p.font()).horizontalAdvance(self._value)
            icons.draw_icon("caution", p, QPointF(x + w + 15, cy), theme.qcolor(val), 16)
