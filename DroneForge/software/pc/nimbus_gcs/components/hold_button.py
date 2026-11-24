"""The big primary action (ARM / TAKE OFF): press and hold; a ring fills; fires `activated` only after the full hold."""
from __future__ import annotations

import os
import time

from PyQt6.QtCore import QRectF, QSize, Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QColor, QLinearGradient, QPainter, QPainterPath, QPen

from PyQt6.QtWidgets import QSizePolicy, QWidget

from .. import theme
from . import icons


def default_hold_ms() -> int:
    try:
        return max(0, int(os.environ.get("DRONEFORGE_HOLD_MS", theme.HOLD_MS_DEFAULT)))
    except ValueError:
        return theme.HOLD_MS_DEFAULT


def dashed_outline(p: QPainter, r: QRectF, rad: float, col):
    path = QPainterPath(); path.addRoundedRect(r.adjusted(.5, .5, -.5, -.5), rad, rad)
    pen = QPen(QColor(col), 1); pen.setStyle(Qt.PenStyle.CustomDashLine); pen.setDashPattern([3, 3])
    p.setPen(pen); p.setBrush(Qt.BrushStyle.NoBrush); p.drawPath(path)


class HoldButton(QWidget):
    activated = pyqtSignal()
    cancelled = pyqtSignal()

    def __init__(self, label="ARM", hint="", icon="drone", hold_ms=None, parent=None):
        super().__init__(parent)
        self._label, self._hint, self._icon = label, hint, icon
        self._hold_ms = hold_ms
        self._enabled, self._reason = True, ""
        self._t0 = None; self._progress = 0.0; self._clock = time.monotonic
        self._timer = QTimer(self); self._timer.setInterval(16); self._timer.timeout.connect(self._tick)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus); self.setMinimumSize(200, 96)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed); self.setFixedHeight(96)

    def sizeHint(self):
        return QSize(376, 96)

    # ---- API
    def hold_ms(self) -> int:
        return default_hold_ms() if self._hold_ms is None else self._hold_ms

    def progress(self) -> float:
        return self._progress

    def is_holding(self) -> bool:
        return self._t0 is not None

    def label(self) -> str:
        return self._label

    def hint(self) -> str:
        return self._hint

    def reason(self) -> str:
        return self._reason

    def set_label(self, t: str):
        self._label = t; self.update()

    def set_hint(self, t: str):
        self._hint = t; self.update()

    def set_enabled_with_reason(self, ok: bool, reason: str = ""):
        self._enabled, self._reason = bool(ok), ("" if ok else reason)
        if not ok:
            self._abort(emit=False)
        self.update()

    def is_ready(self) -> bool:
        return self._enabled

    def trigger(self):
        """Programmatic / test activation (skips the hold). No-op when disabled."""
        if self._enabled:
            self._reset(); self.activated.emit()

    # ---- hold machinery
    def _begin(self):
        if not self._enabled or self._t0 is not None:
            return
        if self.hold_ms() <= 0:
            self.trigger(); return
        self._t0 = self._clock(); self._progress = 0.0; self._timer.start(); self.update()

    def _tick(self):
        if self._t0 is None:
            return
        self._progress = min(1.0, (self._clock() - self._t0) * 1000.0 / self.hold_ms())
        self.update()
        if self._progress >= 1.0:
            self._reset(); self.activated.emit()

    def _reset(self):
        self._timer.stop(); self._t0 = None; self._progress = 0.0; self.update()

    def _abort(self, emit=True):
        if self._t0 is not None:
            self._tick()                      # a release exactly at the end still counts
        if self._t0 is not None:
            self._reset()
            if emit:
                self.cancelled.emit()

    def mousePressEvent(self, e):
        if e.button() == Qt.MouseButton.LeftButton:
            self._begin()

    def mouseReleaseEvent(self, e):
        if e.button() == Qt.MouseButton.LeftButton:
            self._abort()

    def keyPressEvent(self, e):
        if e.key() in (Qt.Key.Key_Space, Qt.Key.Key_Return, Qt.Key.Key_Enter) and not e.isAutoRepeat():
            self._begin()
        elif e.key() not in (Qt.Key.Key_Space, Qt.Key.Key_Return, Qt.Key.Key_Enter):
            super().keyPressEvent(e)

    def keyReleaseEvent(self, e):
        if e.key() in (Qt.Key.Key_Space, Qt.Key.Key_Return, Qt.Key.Key_Enter) and not e.isAutoRepeat():
            self._abort()

    def focusOutEvent(self, e):
        self._abort(); super().focusOutEvent(e)

    # ---- paint
    def paintEvent(self, e):
        p = QPainter(self); p.setRenderHint(QPainter.RenderHint.Antialiasing); p.setRenderHint(QPainter.RenderHint.TextAntialiasing)
        r = QRectF(self.rect()); cy = r.center().y()
        if self._enabled:
            path = QPainterPath(); path.addRoundedRect(r, theme.RADIUS["m"], theme.RADIUS["m"])
            g = QLinearGradient(0, r.top(), 0, r.bottom()); g.setColorAt(0, QColor(theme.ACC).lighter(106)); g.setColorAt(1, QColor(theme.ACC_DEEP).lighter(112))
            p.fillPath(path, g)
            ink, sub = QColor(theme.ON_ACC), QColor(theme.ON_ACC); sub.setAlpha(175)
        else:
            dashed_outline(p, r, theme.RADIUS["m"], theme.HAIR2)
            ink, sub = QColor(theme.DISABLED), QColor(theme.FAINT)
        cx = r.left() + 48
        ring = QRectF(cx - 24, cy - 24, 48, 48)
        track = QColor(ink); track.setAlpha(50 if self._enabled else 120)
        q = QPen(track, 3); p.setPen(q); p.setBrush(Qt.BrushStyle.NoBrush); p.drawEllipse(ring)
        if self._progress > 0:
            q = QPen(ink, 3); q.setCapStyle(Qt.PenCapStyle.RoundCap); p.setPen(q)
            p.drawArc(ring, 90 * 16, -int(360 * 16 * self._progress))
        icons.draw_icon(self._icon, p, ring.center(), ink, 22)
        x = r.left() + 88
        big = r.height() >= 80                                   # compact (short window) variant: smaller label, same layout
        f = theme.f_label(24 if big else 18); f.setWeight(f.Weight.Black); f.setLetterSpacing(f.SpacingType.AbsoluteSpacing, 3.2)
        p.setFont(f); p.setPen(ink); p.drawText(QRectF(x, cy - (36 if big else 28), r.width() - 140, 36 if big else 28), Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignBottom, self._label)
        p.setFont(theme.f_label(10)); p.setPen(sub)
        p.drawText(QRectF(x, cy + (4 if big else 2), r.width() - 100, 24), Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignTop,
                   (self._hint if self._enabled else self._reason) if (self._enabled or self._reason) else "")
        if self._progress > 0:
            p.setFont(theme.f_mono(13)); p.drawText(QRectF(r.left(), cy - 12, r.width() - 24, 24), Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter, f"{int(self._progress * 100)} %")
        if self.hasFocus():
            fp = QPen(QColor(theme.TXT), 1); p.setPen(fp); p.setBrush(Qt.BrushStyle.NoBrush); p.drawRoundedRect(r.adjusted(2.5, 2.5, -2.5, -2.5), 6, 6)
