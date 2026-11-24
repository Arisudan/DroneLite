"""Small picture-in-picture live view (FLY page). Bound to the shared win.video_source; hidden unless live/stalled."""
from __future__ import annotations

import numpy as np
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QImage, QPixmap
from PyQt6.QtWidgets import QLabel, QVBoxLayout, QWidget

from . import theme
from .video_source import get_source


def frame_to_pixmap(bgr, size) -> QPixmap:
    f = np.ascontiguousarray(bgr[:, :, ::-1]); h, w = f.shape[:2]
    img = QImage(f.data, w, h, 3 * w, QImage.Format.Format_RGB888).copy()
    return QPixmap.fromImage(img).scaled(size, Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.FastTransformation)


class VideoPip(QWidget):
    def __init__(self, win) -> None:
        super().__init__()
        self.win = win; self.src = get_source(win)
        self.setFixedSize(320, 180 + 16)
        v = QVBoxLayout(self); v.setContentsMargins(0, 0, 0, 0); v.setSpacing(0)
        self.view = QLabel(); self.view.setFixedSize(320, 180); self.view.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.view.setStyleSheet(f"background:{theme.BG};border:1px solid {theme.HAIR}")
        self.tag = QLabel(""); self.tag.setObjectName("faint"); self.tag.setFont(theme.f_label(10))
        v.addWidget(self.view); v.addWidget(self.tag)
        self._last = -1
        self.setVisible(False)
        self._timer = QTimer(self); self._timer.timeout.connect(self.refresh); self._timer.start(100)

    def refresh(self) -> None:
        s = self.src.state
        show = s in ("live", "stalled")
        if self.isVisible() != show:
            self.setVisible(show)
        if not show:
            self._last = -1; return
        self.tag.setText(s.upper())
        f = self.src.latest()
        if f is not None and self.src.frame_count != self._last:
            self._last = self.src.frame_count
            self.view.setPixmap(frame_to_pixmap(f, self.view.size()))
