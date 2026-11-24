"""Secondary action (DISARM / LAND): icon + label; dashed outline + reason line when disabled."""
from __future__ import annotations

from PyQt6.QtCore import QPointF, QRectF, QSize, Qt, pyqtSignal
from PyQt6.QtGui import QPainter, QPainterPath, QPen

from PyQt6.QtWidgets import QSizePolicy, QWidget

from .. import theme
from . import icons
from .hold_button import dashed_outline


class ActionButton(QWidget):
    clicked = pyqtSignal()

    def __init__(self, label="DISARM", icon="unlock", reason="", parent=None):
        super().__init__(parent)
        self._label, self._icon = label, icon
        self._enabled, self._reason, self._hover, self._down = True, reason, False, False
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus); self.setFixedHeight(72); self.setMinimumWidth(120); self.setMouseTracking(True)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)

    def sizeHint(self):
        return QSize(184, 72)

    def label(self) -> str:
        return self._label

    def reason(self) -> str:
        return self._reason

    def is_ready(self) -> bool:
        return self._enabled

    def set_label(self, t: str):
        self._label = t; self.update()

    def set_enabled_with_reason(self, ok: bool, reason: str = ""):
        self._enabled, self._reason = bool(ok), ("" if ok else reason); self.update()

    def click(self):
        if self._enabled:
            self.clicked.emit()

    def enterEvent(self, e):
        self._hover = True; self.update()

    def leaveEvent(self, e):
        self._hover = False; self._down = False; self.update()

    def mousePressEvent(self, e):
        if e.button() == Qt.MouseButton.LeftButton and self._enabled:
            self._down = True; self.update()

    def mouseReleaseEvent(self, e):
        was = self._down; self._down = False; self.update()
        if was and self.rect().contains(e.position().toPoint()):
            self.clicked.emit()

    def keyPressEvent(self, e):
        if e.key() in (Qt.Key.Key_Space, Qt.Key.Key_Return, Qt.Key.Key_Enter) and not e.isAutoRepeat():
            self.click()
        else:
            super().keyPressEvent(e)

    def paintEvent(self, e):
        p = QPainter(self); p.setRenderHint(QPainter.RenderHint.Antialiasing); p.setRenderHint(QPainter.RenderHint.TextAntialiasing)
        r = QRectF(self.rect())
        if self._enabled:
            path = QPainterPath(); path.addRoundedRect(r.adjusted(.5, .5, -.5, -.5), theme.RADIUS["m"], theme.RADIUS["m"])
            if self._down:
                p.fillPath(path, theme.qcolor(theme.HAIR))
            p.setPen(QPen(theme.qcolor(theme.DIM if self._hover or self.hasFocus() else theme.HAIR2), 1)); p.setBrush(Qt.BrushStyle.NoBrush); p.drawPath(path)
            ink = theme.qcolor(theme.TXT)
        else:
            dashed_outline(p, r, theme.RADIUS["m"], theme.HAIR2); ink = theme.qcolor(theme.FAINT)
        icons.draw_icon(self._icon, p, QPointF(28, 28), ink, 20)
        f = theme.f_label(12); f.setLetterSpacing(f.SpacingType.AbsoluteSpacing, 2.0)
        p.setFont(f); p.setPen(ink); p.drawText(QRectF(52, 16, r.width() - 60, 24), Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter, self._label)
        if self._reason:
            p.setFont(theme.f_body(11)); p.setPen(theme.qcolor(theme.FAINT))
            p.drawText(QRectF(16, 44, r.width() - 24, 20), Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter, self._reason)
