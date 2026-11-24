"""ANALYZE > Live data: every telemetry group (latest value, age, rate) in a table. The 60-second charts were removed from the GUI (archived 2026-11-21)."""
from __future__ import annotations

from PyQt6.QtWidgets import QVBoxLayout, QWidget

from .telemetry_page import TelemetryPage


class LiveDataPage(QWidget):
    def __init__(self, win) -> None:
        super().__init__()
        self.telemetry = TelemetryPage(win)
        l = QVBoxLayout(self); l.setContentsMargins(0, 0, 0, 0); l.setSpacing(16); l.addWidget(self.telemetry, 1)

    def refresh(self, s: dict) -> None:
        self.telemetry.refresh(s)
