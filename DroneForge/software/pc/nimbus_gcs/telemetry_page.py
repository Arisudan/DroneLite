"""ANALYZE > Telemetry: every telemetry group the drone sends, with latest value, age and rate."""
from __future__ import annotations

from PyQt6.QtCore import Qt
from PyQt6.QtGui import QColor
from PyQt6.QtWidgets import QAbstractItemView, QHeaderView, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget

from nimbus_pc import live

from . import theme
from .components import Card

BAD, DIM, GOOD, TEXT, WARN = theme.RED, theme.DIM, theme.ACC, theme.TXT, theme.AMB    # fresh = blue, slow = amber, stale = red (the age text carries it too)


class TelemetryPage(QWidget):
    def __init__(self, win) -> None:
        super().__init__()
        self.win = win
        self.tel = t = QTableWidget(len(live.GROUPS), 4); t.setHorizontalHeaderLabels(["GROUP", "VALUE", "AGE", "HZ"])
        t.verticalHeader().setVisible(False); t.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        t.setSelectionMode(QAbstractItemView.SelectionMode.NoSelection); t.setFont(theme.f_mono(12)); t.setAlternatingRowColors(True); t.setShowGrid(False)
        t.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeMode.Stretch)
        for i, g in enumerate(live.GROUPS):
            it = QTableWidgetItem(g); it.setForeground(QColor(TEXT)); t.setItem(i, 0, it)
            for c in (1, 2, 3):
                t.setItem(i, c, QTableWidgetItem(""))
        t.setColumnWidth(0, 170); t.setColumnWidth(2, 60); t.setColumnWidth(3, 50)
        t.setWordWrap(False); t.setTextElideMode(Qt.TextElideMode.ElideRight); t.verticalHeader().setDefaultSectionSize(26); t.setMinimumHeight(320)
        c = Card("Telemetry groups", "VALUE / AGE / RATE"); c.layout().setContentsMargins(16, 16, 16, 16); c.body.addWidget(t, 1)
        l = QVBoxLayout(self); l.setContentsMargins(0, 0, 0, 0); l.addWidget(c, 1)

    def refresh(self, s: dict) -> None:
        st = self.win.model.state
        for i, g in enumerate(live.GROUPS):
            name = live.DECODED.get(g)
            age = st.age(name) if name else None
            if age is None:
                ts = [st.in_times[t][-1] for t in live.GROUP_TYPES.get(g, []) if st.in_times.get(t)]
                age = (st.clock() - max(ts)) if ts else None
            hz = sum(st.rate(st.in_times, t) for t in live.GROUP_TYPES.get(g, []))
            txt = live.summarize(g, st)
            self.tel.item(i, 1).setText(txt); self.tel.item(i, 1).setToolTip(txt)
            ai = self.tel.item(i, 2); ai.setText("--" if age is None else f"{age:.1f}s")
            ai.setForeground(QColor(DIM if age is None else GOOD if age < 1 else WARN if age < 3 else BAD))
            self.tel.item(i, 3).setText(f"{hz:.1f}")
