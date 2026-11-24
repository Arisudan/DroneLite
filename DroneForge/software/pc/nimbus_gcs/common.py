"""Small shared UI helpers (kept out of main_window.py so pages can import them without a circular import)."""
from __future__ import annotations

from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import QFrame, QHBoxLayout, QLabel, QToolButton, QVBoxLayout, QWidget

UI_FONT = ["IBM Plex Sans", "Inter", "Segoe UI", "DejaVu Sans", "sans-serif"]
MONO_FONT = ["IBM Plex Mono", "JetBrains Mono", "DejaVu Sans Mono", "monospace"]


def mono(size: int = 9, bold: bool = False) -> QFont:
    f = QFont(); f.setFamilies(MONO_FONT); f.setPointSize(size); f.setBold(bold); return f


def card(title: str, collapsible: bool = False, collapsed: bool = False):
    """A titled panel. Returns (frame, body_layout)."""
    fr = QFrame(); fr.setObjectName("card")
    outer = QVBoxLayout(fr); outer.setContentsMargins(14, 12, 14, 14); outer.setSpacing(8)
    head = QHBoxLayout(); t = QLabel(title.upper()); t.setObjectName("cardTitle"); head.addWidget(t); head.addStretch(1)
    body = QWidget(); bl = QVBoxLayout(body); bl.setContentsMargins(0, 0, 0, 0); bl.setSpacing(8)
    if collapsible:
        btn = QToolButton(); btn.setText("show" if collapsed else "hide"); btn.setCheckable(True); btn.setChecked(not collapsed)
        from . import theme as _t
        btn.setStyleSheet(f"QToolButton{{border:0;color:{_t.DIM};}} QToolButton:hover{{color:{_t.TXT};}}")
        def tog(on, b=btn, w=body): w.setVisible(on); b.setText("hide" if on else "show")
        btn.toggled.connect(tog); body.setVisible(not collapsed); head.addWidget(btn)
    outer.addLayout(head); outer.addWidget(body); outer.addStretch(1)          # extra height goes below the content, never around it
    return fr, bl
