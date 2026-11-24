"""Text tabs with a 2-px accent underline."""
from __future__ import annotations

from PyQt6.QtCore import QPointF, QRectF, Qt, pyqtSignal
from PyQt6.QtGui import QFontMetricsF, QPainter

from PyQt6.QtWidgets import QWidget

from .. import theme


class TopTabs(QWidget):
    changed = pyqtSignal(int)

    def __init__(self, labels=(), parent=None):
        super().__init__(parent)
        self._labels = list(labels); self._i = 0; self._hover = -1
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus); self.setMouseTracking(True); self.setFixedHeight(theme.TOP_BAR_H)
        self.setMinimumWidth(int(self._layout()[-1][1]) if self._labels else 0)

    def _layout(self):
        fm = QFontMetricsF(theme.f_label(12)); x = 0.0; out = []
        for t in self._labels:
            w = fm.horizontalAdvance(t.upper()) + 40; out.append((x, x + w)); x += w
        return out or [(0, 0)]

    def sizeHint(self):
        from PyQt6.QtCore import QSize
        return QSize(int(self._layout()[-1][1]), theme.TOP_BAR_H)

    def current(self) -> int:
        return self._i

    def select(self, i: int):
        if 0 <= i < len(self._labels) and i != self._i:
            self._i = i; self.update(); self.changed.emit(i)

    def keyPressEvent(self, e):
        if e.key() == Qt.Key.Key_Left:
            self.select(max(0, self._i - 1))
        elif e.key() == Qt.Key.Key_Right:
            self.select(min(len(self._labels) - 1, self._i + 1))
        else:
            super().keyPressEvent(e)

    def _at(self, x):
        for i, (a, b) in enumerate(self._layout()):
            if a <= x < b and i < len(self._labels):
                return i
        return -1

    def mouseMoveEvent(self, e):
        self._hover = self._at(e.position().x()); self.update()

    def leaveEvent(self, e):
        self._hover = -1; self.update()

    def mousePressEvent(self, e):
        i = self._at(e.position().x())
        if i >= 0:
            self.select(i)

    def paintEvent(self, e):
        p = QPainter(self); p.setRenderHint(QPainter.RenderHint.TextAntialiasing); p.setFont(theme.f_label(12))
        for i, ((a, b), t) in enumerate(zip(self._layout(), self._labels)):
            sel = i == self._i
            p.setPen(theme.qcolor(theme.TXT if sel or i == self._hover else theme.DIM))
            p.drawText(QRectF(a, 0, b - a, self.height() - 2), Qt.AlignmentFlag.AlignCenter, t.upper())
            if sel:
                p.fillRect(QRectF(a + 12, self.height() - 2, b - a - 24, 2), theme.qcolor(theme.ACC))
        if self.hasFocus():
            p.setPen(theme.qcolor(theme.HAIR2)); p.drawLine(0, self.height() - 1, self.width(), self.height() - 1)
