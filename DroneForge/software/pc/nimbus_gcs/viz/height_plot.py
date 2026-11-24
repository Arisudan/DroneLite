"""HeightPlot: TARGET / ACTUAL / RANGE time series over time."""
from __future__ import annotations

import math
from collections import deque

from PyQt6.QtCore import QPointF, QRectF, QSize, Qt
from PyQt6.QtGui import QLinearGradient, QPainterPath, QPen
from PyQt6.QtWidgets import QWidget

from .. import theme as T
from . import col, draw_text, fmt_mmss, hline, new_painter, text_w, vline


def nice_ticks(vmax: float, n: int = 3):
    raw = vmax / n; mag = 10 ** math.floor(math.log10(raw)); step = mag
    for m in (1, 2, 2.5, 5, 10):
        step = m * mag
        if step >= raw:
            break
    k = math.ceil(vmax / step - 1e-9)
    return [round(i * step, 6) for i in range(k + 1)], step


class HeightPlot(QWidget):
    LEFT, RIGHT = 72, 24

    def __init__(self, parent=None, window: float = 110.0):
        super().__init__(parent)
        T.load_fonts()
        self._win = float(window)
        self._buf: deque = deque(maxlen=60000)     # (t, target, actual, range)
        self._plan: list = []                       # optional planned target [(t, v)], drawn dashed behind the live series
        self.setMinimumHeight(160)

    def plot_margins(self) -> tuple[int, int]:
        return (self.LEFT, self.RIGHT)

    # ---- data
    def set_window(self, seconds: float) -> None:
        self._win = max(1.0, float(seconds)); self._trim(); self.update()

    def window(self) -> float:
        return self._win

    def push(self, t: float, target, actual, rng) -> None:
        self._buf.append((float(t), target, actual, rng)); self._trim(); self.update()

    def _trim(self) -> None:
        if self._buf:
            lim = self._buf[-1][0] - self._win - 1.0
            while self._buf and self._buf[0][0] < lim:
                self._buf.popleft()

    def set_plan(self, pts) -> None:
        self._plan = [(float(a), float(b)) for a, b in pts]; self.update()

    def clear(self) -> None:
        self._buf.clear(); self.update()

    def load_series(self, t, target, actual, rng) -> None:
        self._buf.clear()
        for row in zip(t, target, actual, rng):
            self._buf.append(tuple(row))
        self._trim(); self.update()

    def points(self) -> list:
        return list(self._buf)

    def view_range(self) -> tuple[float, float]:
        last = self._buf[-1][0] if self._buf else 0.0
        t0 = max(0.0, last - self._win)
        return t0, t0 + self._win

    def sizeHint(self) -> QSize:
        return QSize(900, 260)

    # ---- paint
    def paintEvent(self, _):
        p = new_painter(self); W, H = self.width(), self.height()
        px0, px1 = self.LEFT, W - self.RIGHT
        py0, py1 = 36.0, H - 28.0
        t0, t1 = self.view_range()
        vals = [v for r in self._buf for v in r[1:] if v is not None]
        vmax = max(0.75, max(vals) * 1.1 if vals else 0.75)
        ticks, _ = nice_ticks(vmax); vmax = ticks[-1]
        X = lambda t: px0 + (t - t0) / (t1 - t0) * (px1 - px0)
        Y = lambda v: py1 - v / vmax * (py1 - py0)
        draw_text(p, 24, py0 - 20, "HEIGHT m", T.f_label(10), T.FAINT)
        for v in ticks:
            yy = round(Y(v))
            hline(p, px0, px1, yy, T.HAIR2 if v == 0 else T.HAIR)
            draw_text(p, px0 - 16, yy + 4, f"{v:.2f}", T.f_mono(11), T.FAINT, "r")
        step = 10 if self._win <= 150 else 30 if self._win <= 600 else 60
        k = math.ceil(t0 / step) * step
        while k <= t1 + 1e-6:
            xx = round(X(k)); p.fillRect(QRectF(xx, py1, 1, 4), col(T.HAIR2))
            if (k // step) % 2 == 0:
                draw_text(p, xx + .5, py1 + 20, fmt_mmss(k), T.f_mono(10), T.FAINT, "c")
            k += step
        # legend (top right)
        lx = px1; ly = py0 - 24; lf = T.f_label(10)
        for name, kind in (("RANGE", "rng"), ("ACTUAL", "act"), ("TARGET", "tgt")):
            draw_text(p, lx, ly + 4, name, lf, T.DIM, "r"); lx -= text_w(name, lf) + 8
            p.setPen(self._series_pen(kind)); p.drawLine(QPointF(lx - 22, ly), QPointF(lx, ly)); lx -= 46
        # series
        p.save(); p.setClipRect(QRectF(px0, py0 - 4, px1 - px0, py1 - py0 + 8))
        def path(idx):
            pa = QPainterPath(); pen_up = False
            for r in self._buf:
                v = r[idx]
                if v is None:
                    pen_up = False; continue
                pt = QPointF(X(r[0]), Y(v))
                if pen_up: pa.lineTo(pt)
                else: pa.moveTo(pt); pen_up = True
            return pa
        if self._plan:
            pp = QPainterPath(); pp.moveTo(X(self._plan[0][0]), Y(self._plan[0][1]))
            for a, b in self._plan[1:]:
                pp.lineTo(X(a), Y(b))
            p.setBrush(Qt.BrushStyle.NoBrush); p.setPen(self._series_pen("tgt")); p.drawPath(pp)
        pa = path(2)
        if not pa.isEmpty():
            area = QPainterPath(pa); area.lineTo(pa.currentPosition().x(), Y(0)); area.lineTo(pa.elementAt(0).x, Y(0)); area.closeSubpath()
            g = QLinearGradient(0, py0, 0, py1); g.setColorAt(0, col(T.ACC, 30)); g.setColorAt(1, col(T.ACC, 0))
            p.fillPath(area, g)
        p.setBrush(Qt.BrushStyle.NoBrush)
        for idx, kind in ((3, "rng"), (1, "tgt"), (2, "act")):
            p.setPen(self._series_pen(kind)); p.drawPath(path(idx))
        p.restore()
        if self._buf:
            last = self._buf[-1]; xn = round(X(last[0])) + .5
            p.setPen(QPen(col(T.ACC), 1)); p.drawLine(QPointF(xn, py0), QPointF(xn, py1))
            if last[2] is not None:
                p.setPen(Qt.PenStyle.NoPen); p.setBrush(col(T.ACC)); p.drawEllipse(QPointF(xn, Y(last[2])), 3.5, 3.5)
        p.end()

    @staticmethod
    def _series_pen(kind: str) -> QPen:
        if kind == "tgt":
            q = QPen(col(T.TXT, 200), 1.25); q.setStyle(Qt.PenStyle.CustomDashLine); q.setDashPattern([5, 4]); return q
        if kind == "act":
            return QPen(col(T.ACC), 1.75)
        return QPen(col(T.DIM, 170), 1)
