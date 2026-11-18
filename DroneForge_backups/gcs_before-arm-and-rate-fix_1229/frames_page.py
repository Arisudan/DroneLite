"""ANALYZE > Radio frames: raw radio frames in and out, with counts and rates."""
from __future__ import annotations

from PyQt6.QtWidgets import QAbstractItemView, QHeaderView, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget

from nimbus_pc import live

from . import theme
from .components import Card


class FramesPage(QWidget):
    def __init__(self, win) -> None:
        super().__init__()
        self.win = win
        self.frames = t = QTableWidget(0, 8); t.setHorizontalHeaderLabels(["IN", "NAME", "COUNT", "HZ", "AGE", "OUT", "COUNT", "HZ"])
        t.verticalHeader().setVisible(False); t.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        t.setFont(theme.f_mono(12)); t.setAlternatingRowColors(True); t.setShowGrid(False); t.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        for col in (1, 5):
            t.horizontalHeader().setSectionResizeMode(col, QHeaderView.ResizeMode.ResizeToContents)
        t.setWordWrap(False); t.verticalHeader().setDefaultSectionSize(26); t.setMinimumHeight(320)
        c = Card("Radio frames", "IN LEFT / OUT RIGHT"); c.layout().setContentsMargins(16, 16, 16, 16); c.body.addWidget(t, 1)
        l = QVBoxLayout(self); l.setContentsMargins(0, 0, 0, 0); l.addWidget(c, 1)

    def refresh(self, s: dict) -> None:
        st = self.win.model.state
        ins = sorted(st.in_count.items(), key=lambda kv: -st.rate(st.in_times, kv[0]))[:12]
        outs = sorted(st.out_count.items(), key=lambda kv: -kv[1])[:12]
        n = max(len(ins), len(outs))
        self.frames.setRowCount(n)
        for r in range(n):
            vals = [""] * 8
            if r < len(ins):
                t, cnt = ins[r]; q = st.in_times.get(t)
                vals[0:5] = [f"0x{t:02X}", live.TYPE_INFO.get(t, ("", "?"))[1], str(cnt), f"{st.rate(st.in_times, t):.1f}", f"{st.clock() - q[-1]:.1f}s" if q else "--"]
            if r < len(outs):
                t, cnt = outs[r]
                vals[5:8] = [f"0x{t:02X} {live.TYPE_INFO.get(t, ('', '?'))[1]}", str(cnt), f"{st.rate(st.out_times, t):.1f}"]
            for c_i, text in enumerate(vals):
                it = self.frames.item(r, c_i)
                if it is None:
                    it = QTableWidgetItem(); self.frames.setItem(r, c_i, it)
                it.setText(text)
