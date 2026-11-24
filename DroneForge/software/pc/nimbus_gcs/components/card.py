"""Hairline card with optional small-caps title and right-aligned meta text."""
from __future__ import annotations

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QFrame, QHBoxLayout, QLabel, QVBoxLayout

from .. import theme


class Card(QFrame):
    def __init__(self, title: str = "", meta: str = "", parent=None):
        super().__init__(parent)
        self.setObjectName("card")
        outer = QVBoxLayout(self); outer.setContentsMargins(24, 16, 24, 16); outer.setSpacing(8)
        self._head = QHBoxLayout(); self._head.setContentsMargins(0, 0, 0, 0)
        self._title = QLabel(title.upper()); self._title.setObjectName("label"); self._title.setFont(theme.f_label())
        self._meta = QLabel(meta); self._meta.setObjectName("faint"); self._meta.setFont(theme.f_mono(11))
        self._meta.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        self._head.addWidget(self._title); self._head.addStretch(1); self._head.addWidget(self._meta)
        outer.addLayout(self._head)
        self.body = QVBoxLayout(); self.body.setContentsMargins(0, 0, 0, 0); self.body.setSpacing(8)
        outer.addLayout(self.body, 1)
        self._sync()

    def _sync(self):
        self._title.setVisible(bool(self._title.text())); self._meta.setVisible(bool(self._meta.text()))

    def set_title(self, t: str):
        self._title.setText(t.upper()); self._sync()

    def set_meta(self, t: str):
        self._meta.setText(t); self._sync()

    def title(self) -> str:
        return self._title.text()

    def meta(self) -> str:
        return self._meta.text()
