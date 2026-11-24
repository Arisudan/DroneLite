"""`-  0.50 m  +` setpoint field: Thin 72-px numeral, step, min/max, unit, range caption, long-press repeat."""
from __future__ import annotations

from PyQt6.QtCore import QPointF, QRectF, QSize, Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QFontMetricsF, QPainter, QPainterPath, QPen

from PyQt6.QtWidgets import QWidget

from .. import theme
from . import icons


class StepField(QWidget):
    valueChanged = pyqtSignal(float)

    def __init__(self, value=0.5, step=0.05, minimum=0.25, maximum=1.5, unit="m", decimals=2, parent=None):
        super().__init__(parent)
        self._min, self._max, self._step, self._unit, self._dec = minimum, maximum, step, unit, decimals
        self._v = min(max(value, minimum), maximum); self._dir = 0; self._down = 0; self._enabled = True
        self._rep = QTimer(self); self._rep.setInterval(90); self._rep.timeout.connect(self._repeat)
        self._delay = QTimer(self); self._delay.setSingleShot(True); self._delay.setInterval(450); self._delay.timeout.connect(self._rep.start)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus); self.setMinimumSize(300, 96)

    def sizeHint(self):
        return QSize(328, 100)

    def value(self) -> float:
        return self._v

    def set_value(self, v: float, emit: bool = False):
        v = round(min(max(float(v), self._min), self._max), 6)
        if v != self._v:
            self._v = v; self.update()
            if emit:
                self.valueChanged.emit(v)

    def set_range(self, lo, hi):
        self._min, self._max = lo, hi; self.set_value(self._v); self.update()

    def set_enabled_state(self, ok: bool):
        self._enabled = ok; self.update()

    def text(self) -> str:
        return f"{self._v:.{self._dec}f}"

    def range_text(self) -> str:
        return f"{self._min:.{self._dec}f} – {self._max:.{self._dec}f} {self._unit}"

    def step_by(self, n: int):
        if self._enabled:
            self.set_value(self._v + n * self._step, emit=True)

    def _btn_rects(self):
        cy = self.height() / 2
        return QRectF(0, cy - 24, 48, 48), QRectF(self.width() - 48, cy - 24, 48, 48)

    def _repeat(self):
        self.step_by(self._dir)

    def mousePressEvent(self, e):
        m, p = self._btn_rects(); pt = e.position()
        d = -1 if m.contains(pt) else 1 if p.contains(pt) else 0
        if d and e.button() == Qt.MouseButton.LeftButton:
            self._dir, self._down = d, d; self.step_by(d); self._delay.start(); self.update()

    def mouseReleaseEvent(self, e):
        self._delay.stop(); self._rep.stop(); self._dir = self._down = 0; self.update()

    def keyPressEvent(self, e):
        k = e.key()
        if k in (Qt.Key.Key_Up, Qt.Key.Key_Right, Qt.Key.Key_Plus):
            self.step_by(1)
        elif k in (Qt.Key.Key_Down, Qt.Key.Key_Left, Qt.Key.Key_Minus):
            self.step_by(-1)
        else:
            super().keyPressEvent(e)

    def paintEvent(self, e):
        p = QPainter(self); p.setRenderHint(QPainter.RenderHint.Antialiasing); p.setRenderHint(QPainter.RenderHint.TextAntialiasing)
        w, h = self.width(), self.height(); cy = (h - 16) / 2
        m, pl = QRectF(0, cy - 24, 48, 48), QRectF(w - 48, cy - 24, 48, 48)
        for r, name, d, lim in ((m, "minus", -1, self._v <= self._min), (pl, "plus", 1, self._v >= self._max)):
            path = QPainterPath(); path.addRoundedRect(r.adjusted(.5, .5, -.5, -.5), 24, 24)
            if self._down == d:
                p.fillPath(path, theme.qcolor(theme.HAIR))
            dim = lim or not self._enabled
            q = QPen(theme.qcolor(theme.HAIR2), 1)
            if dim:
                q.setStyle(Qt.PenStyle.CustomDashLine); q.setDashPattern([3, 3])
            p.setPen(q); p.setBrush(Qt.BrushStyle.NoBrush); p.drawPath(path)
            icons.draw_icon(name, p, r.center(), theme.qcolor(theme.DISABLED if dim else theme.TXT), 20)
        f = theme.f_xl(72 if h >= 96 else 40); fm = QFontMetricsF(f); nw = fm.horizontalAdvance(self.text())
        uf = theme.f_l(18); uw = QFontMetricsF(uf).horizontalAdvance(self._unit)
        x0 = (w - nw - 6 - uw) / 2
        p.setFont(f); p.setPen(theme.qcolor(theme.TXT if self._enabled else theme.DISABLED))
        base = cy + (fm.ascent() - fm.descent()) / 2 + 2
        p.drawText(QPointF(x0, base), self.text())
        p.setFont(uf); p.setPen(theme.qcolor(theme.DIM)); p.drawText(QPointF(x0 + nw + 6, base), self._unit)
        p.setFont(theme.f_mono(10)); p.setPen(theme.qcolor(theme.FAINT))
        p.drawText(QRectF(0, h - 18, w, 16), Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
                   f"STEP {self._step:g} {self._unit}   ·   {self.range_text()}")
