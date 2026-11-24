"""Non-modal toasts anchored bottom-centre of the parent. Queue of up to 3, fade in/out, optional Undo."""
from __future__ import annotations

from PyQt6.QtCore import QEvent, QPropertyAnimation, Qt, QTimer
from PyQt6.QtWidgets import QFrame, QGraphicsOpacityEffect, QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget

from .. import theme

MAX_TOASTS = 3


class _Toast(QFrame):
    def __init__(self, host, message, undo, timeout_ms):
        super().__init__(host)
        self.host, self.undo = host, undo
        self.setObjectName("card"); self.setStyleSheet(f"QFrame#card {{ background: {theme.PANEL}; border: 1px solid {theme.HAIR2}; border-radius: {theme.RADIUS['m']}px; }}")
        lay = QHBoxLayout(self); lay.setContentsMargins(16, 10, 12, 10); lay.setSpacing(16)
        self.label = QLabel(message); self.label.setFont(theme.f_body(13)); lay.addWidget(self.label)
        self.undo_button = None
        if callable(undo):
            b = QPushButton("UNDO"); b.setObjectName("ghost"); b.setStyleSheet(f"color: {theme.ACC}; font-weight: 700; letter-spacing: 1.4px; padding: 4px 8px;")
            b.setCursor(Qt.CursorShape.PointingHandCursor); b.clicked.connect(self._undo); lay.addWidget(b); self.undo_button = b
        self.fx = QGraphicsOpacityEffect(self); self.setGraphicsEffect(self.fx); self.fx.setOpacity(0.0)
        self.anim = QPropertyAnimation(self.fx, b"opacity", self); self.anim.setDuration(180)
        self.anim.setStartValue(0.0); self.anim.setEndValue(1.0); self.anim.start()
        self.timer = QTimer(self); self.timer.setSingleShot(True); self.timer.timeout.connect(self.dismiss)
        if timeout_ms > 0:
            self.timer.start(int(timeout_ms))

    def _undo(self):
        try:
            self.undo()
        finally:
            self.dismiss()

    def dismiss(self):
        if getattr(self, "_gone", False):
            return
        self._gone = True; self.timer.stop(); self.hide(); self.host._remove(self)


class ToastHost(QWidget):
    def __init__(self, parent: QWidget):
        super().__init__(parent)
        self.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, False)
        self._col = QVBoxLayout(self); self._col.setContentsMargins(0, 0, 0, 0); self._col.setSpacing(8)
        self._toasts: list[_Toast] = []
        parent.installEventFilter(self); self.raise_(); self._place()

    def count(self) -> int:
        return len(self._toasts)

    def messages(self) -> list[str]:
        return [t.label.text() for t in self._toasts]

    def show_toast(self, message: str, undo=None, timeout_ms: int = 5000) -> _Toast:
        while len(self._toasts) >= MAX_TOASTS:
            self._toasts[0].dismiss()
        t = _Toast(self, message, undo, timeout_ms); self._toasts.append(t); self._col.addWidget(t)
        self.show(); self.raise_(); t.show(); self._place(); return t

    def _remove(self, t):
        if t in self._toasts:
            self._toasts.remove(t)
        self._col.removeWidget(t); t.deleteLater()
        if not self._toasts:
            self.hide()
        self._place()

    def eventFilter(self, o, e):
        if e.type() == QEvent.Type.Resize:
            self._place()
        return False

    def _place(self):
        par = self.parentWidget()
        if par is None:
            return
        self.adjustSize(); w, h = self.sizeHint().width(), self.sizeHint().height()
        self.setGeometry((par.width() - w) // 2, par.height() - h - 24, w, h)
