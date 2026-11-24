"""Post-flight analysis view: findings (coloured by level) and four stacked static charts on one shared time axis. Decoding runs in a worker thread.
No dialogs, no tick boxes: everything is inline."""
from __future__ import annotations

import os
import threading
from typing import List, Optional, Tuple

from PyQt6.QtCore import QRectF, Qt, pyqtSignal
from PyQt6.QtGui import QColor, QPainter, QPen, QPolygonF
from PyQt6.QtCore import QPointF
from PyQt6.QtWidgets import QFrame, QLabel, QSizePolicy, QVBoxLayout, QWidget

from . import theme
from .components import Card

DIM, GRID, PANEL, TEXT = QColor(theme.DIM), QColor(theme.HAIR), QColor(theme.PANEL), QColor(theme.TXT)
# findings: the symbol carries the level, colour only reinforces it (grey ok, amber caution, red alarm)
LEVEL = {"ok": ("✔", theme.DIM), "warn": ("⚠", theme.AMB), "bad": ("✖", theme.RED)}
Series = Tuple[str, str, QColor]
S = Qt.PenStyle
# Series look: greys + the single blue; lines differ by dash pattern so they also read without colour.
SERIES_LOOK = [(QColor(theme.ACC), S.SolidLine), (QColor(theme.TXT), S.SolidLine), (QColor(theme.DIM), S.DashLine),
               (QColor(theme.ACC_DEEP), S.DotLine), (QColor(theme.FAINT), S.DashDotLine)]


def look(i: int):
    return SERIES_LOOK[i % len(SERIES_LOOK)]


class StaticChart(QWidget):
    """A static line chart: x = seconds (shared range set by the caller), y auto-scaled (or fixed)."""

    def __init__(self, title: str, series: List[Series], unit: str = "", ymin: Optional[float] = None, ymax: Optional[float] = None, parent=None) -> None:
        super().__init__(parent)
        self.title, self.series, self.unit, self.ymin, self.ymax = title, series, unit, ymin, ymax
        self.t: List[float] = []; self.data: dict = {}; self.xmax = 1.0; self.show_axis = False
        self.setMinimumHeight(88); self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

    def set_data(self, t: List[float], data: dict, xmax: float, show_axis: bool = False) -> None:
        self.t, self.data, self.xmax, self.show_axis = t, data, max(xmax, 1e-6), show_axis
        self.update()

    def paintEvent(self, _) -> None:
        p = QPainter(self); p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height(); p.fillRect(self.rect(), PANEL)
        left, right, top, bottom = 46, 10, 20, 22 if self.show_axis else 6
        plot = QRectF(left, top, max(w - left - right, 10), max(h - top - bottom, 10))
        p.setFont(theme.f_mono(10))
        vals = [v for k, _, _ in self.series for v in self.data.get(k, [])]
        lo = self.ymin if self.ymin is not None else (min(vals) if vals else 0.0)
        hi = self.ymax if self.ymax is not None else (max(vals) if vals else 1.0)
        if hi - lo < 1e-9: lo, hi = lo - 1, hi + 1
        pad = 0.05 * (hi - lo) if self.ymin is None else 0.0; lo -= pad; hi += pad
        p.setPen(QPen(GRID, 1))
        for i in range(5):
            y = plot.top() + plot.height() * i / 4; p.drawLine(QPointF(plot.left(), y), QPointF(plot.right(), y))
            p.setPen(DIM); p.drawText(QRectF(0, y - 8, left - 4, 16), int(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter), f"{hi - (hi - lo) * i / 4:.3g}"); p.setPen(QPen(GRID, 1))
        step = next((s for s in (1, 2, 5, 10, 20, 30, 60) if self.xmax / s <= 12), 120)
        for k in range(int(self.xmax // step) + 1):
            x = plot.left() + plot.width() * k * step / self.xmax
            p.setPen(QPen(GRID, 1)); p.drawLine(QPointF(x, plot.top()), QPointF(x, plot.bottom()))
            if self.show_axis: p.setPen(DIM); p.drawText(QRectF(x - 20, plot.bottom() + 3, 40, 14), int(Qt.AlignmentFlag.AlignCenter), f"{k * step:g} s")
        x_of = lambda tv: plot.left() + plot.width() * tv / self.xmax
        y_of = lambda v: plot.bottom() - plot.height() * (v - lo) / (hi - lo)
        for n, (key, _, col) in enumerate(self.series):
            ys = self.data.get(key)
            if not ys or len(ys) != len(self.t): continue
            p.setPen(QPen(col, 1.6, look(n)[1])); p.drawPolyline(QPolygonF([QPointF(x_of(a), min(max(y_of(b), plot.top()), plot.bottom())) for a, b in zip(self.t, ys)]))
        p.setFont(theme.f_label(10)); p.setPen(TEXT); p.drawText(QRectF(left, 2, 260, 16), int(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter), (self.title + (f" ({self.unit})" if self.unit else "")).upper())
        x = w - right
        for n, (key, label, col) in reversed(list(enumerate(self.series))):
            tw = p.fontMetrics().horizontalAdvance(label) + 32; x -= tw
            p.setPen(QPen(col, 2, look(n)[1])); p.drawLine(QPointF(x, 10), QPointF(x + 18, 10)); p.setPen(DIM); p.drawText(QPointF(x + 22, 14), label)
        p.end()


class AnalysisView(QWidget):
    sig_done = pyqtSignal(str, object)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        l = QVBoxLayout(self); l.setContentsMargins(0, 0, 0, 0); l.setSpacing(theme.SP["m"])
        self.summary_card = Card("Findings")
        self.title = QLabel("Select a log and press Analyze, or open a saved log: the findings and charts appear here."); self.title.setWordWrap(True); self.title.setFont(theme.f_body(13))
        self.findings = QLabel(""); self.findings.setTextFormat(Qt.TextFormat.RichText); self.findings.setWordWrap(True); self.findings.setObjectName("mono"); self.findings.setFont(theme.f_mono(13))
        self.summary_card.body.addWidget(self.title); self.summary_card.body.addWidget(self.findings)
        l.addWidget(self.summary_card)
        C = QColor
        k = lambda i: look(i)[0]
        self.charts = [
            StaticChart("Motors and throttle", [("M1", "M1", k(0)), ("M2", "M2", k(1)), ("M3", "M3", k(2)), ("M4", "M4", k(3)), ("throttle", "throttle", k(4))], "fraction", 0.0, 1.0),
            StaticChart("Height (range sensor)", [("range", "range m", k(0))], "m"),
            StaticChart("Gyro rates", [("gyro_roll", "roll", k(0)), ("gyro_pitch", "pitch", k(2))], "deg/s"),
            StaticChart("Battery", [("vbat", "V", k(1))], "V"),
        ]
        self.charts_card = Card("Charts", "SHARED TIME AXIS")
        self.charts_card.layout().setContentsMargins(16, 16, 16, 16)
        for c in self.charts: self.charts_card.body.addWidget(c, 1)
        l.addWidget(self.charts_card, 1)
        self.token = 0
        self.sig_done.connect(self._finish)

    def show_log(self, path: str) -> None:
        self.token += 1; tok = self.token
        try: size = os.path.getsize(path)
        except OSError: size = -1
        name = os.path.basename(path)
        if size == 0:
            self._text(f"{name} is empty (0 bytes): nothing was recorded in it.", [], False); return
        self._text(f"Decoding {name}...", [], False)
        threading.Thread(target=self._work, args=(path, tok), daemon=True).start()

    def _work(self, path: str, tok: int) -> None:
        try:
            from . import bfl
            lg = bfl.Log(path); lg.decode(); res = bfl.analyze(lg)
        except Exception as e:                                   # a damaged log must produce a message, never a crash
            res = {"ok": False, "findings": [("bad", f"Could not decode: {e}")], "t": [], "series": {}}
        try: self.sig_done.emit(f"{tok}|{os.path.basename(path)}", res)
        except RuntimeError: pass                                # the view was closed meanwhile

    def _text(self, title: str, findings: list, charts: bool) -> None:
        self.title.setText(title)
        rows = []
        for lv, tx in findings:
            sym, col = LEVEL.get(lv, LEVEL["ok"]); tx = tx.replace("&", "&amp;").replace("<", "&lt;")
            rows.append(f'<span style="color:{col}">{sym}&nbsp;{tx}</span>')
        self.findings.setText("<br>".join(rows)); self.findings.setVisible(bool(rows))
        if not charts:
            for c in self.charts: c.set_data([], {}, 1.0)

    def _finish(self, tag: str, res: dict) -> None:
        tok, _, name = tag.partition("|")
        if int(tok) != self.token: return
        self._text(name + (f"   ({res.get('duration', 0):.1f} s)" if res.get("ok") else ""), res["findings"], True)
        t, s = res["t"], dict(res["series"]); xmax = t[-1] if t else 1.0
        if "range" in s: s["range"] = [v / 100.0 for v in s["range"]]      # log is in cm; the GCS shows metres
        if "mean" in s: s["mean"] = [v * 100 for v in s["mean"]]
        for i, c in enumerate(self.charts): c.set_data(t, s, xmax, show_axis=(i == len(self.charts) - 1))
