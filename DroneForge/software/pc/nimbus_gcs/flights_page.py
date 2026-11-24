"""ANALYZE > Flights: the FC logs (download, analyse). The PC-recording replay was removed from the GUI (archived 2026-11-21): not part of the camera-flight pipeline."""
from __future__ import annotations

from PyQt6.QtWidgets import QVBoxLayout, QWidget

from .logs_tab import LogsTab


class FlightsPage(QWidget):
    def __init__(self, win) -> None:
        super().__init__()
        self.logs = LogsTab(win)
        l = QVBoxLayout(self); l.setContentsMargins(0, 0, 0, 0); l.setSpacing(12); l.addWidget(self.logs, 1)

    def refresh(self, s: dict) -> None:
        self.logs.refresh()
