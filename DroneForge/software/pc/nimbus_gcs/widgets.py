"""Custom-painted widgets (PyQt6 only, no QtCharts): artificial horizon, strip chart, RC channel bars, status LED."""
from __future__ import annotations

import math
from typing import Optional

from PyQt6.QtCore import QPointF, QRectF, Qt
from PyQt6.QtGui import QBrush, QColor, QFont, QPainter, QPainterPath, QPen, QPolygonF
from PyQt6.QtWidgets import QSizePolicy, QWidget

from . import theme as _t

# the chart widgets draw with the theme tokens (greys + the one accent; amber caution, red alarm): no private palette
BG, PANEL, GRID, TEXT, DIM = QColor(_t.PANEL), QColor(_t.HAIR), QColor(_t.HAIR2), QColor(_t.TXT), QColor(_t.DIM)
GOOD, WARN, BAD, ACCENT = QColor(_t.ACC), QColor(_t.AMB), QColor(_t.RED), QColor(_t.ACC_DEEP)


class AttitudeWidget(QWidget):
    """Artificial horizon. Angles in degrees; pitch up = positive, roll right = positive."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.pitch = self.roll = self.yaw = 0.0
        self.valid = False
        self.setMinimumSize(190, 190)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

    def set_attitude(self, pitch: Optional[float], roll: Optional[float], yaw: Optional[float]) -> None:
        self.valid = pitch is not None and roll is not None
        self.pitch, self.roll, self.yaw = pitch or 0.0, roll or 0.0, yaw or 0.0
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), BG)
        w, h = self.width(), self.height()
        r = min(w, h) / 2 - 8
        cx, cy = w / 2, h / 2
        clip = QPainterPath(); clip.addEllipse(QPointF(cx, cy), r, r)
        p.save(); p.setClipPath(clip); p.translate(cx, cy); p.rotate(-self.roll)
        k = r / 40.0                                              # pixels per degree of pitch
        dy = self.pitch * k
        p.fillRect(QRectF(-2 * r, -2 * r + dy, 4 * r, 2 * r), QColor("#2f6fb0"))
        p.fillRect(QRectF(-2 * r, dy, 4 * r, 2 * r), QColor("#7a5230"))
        p.setPen(QPen(QColor("#ffffff"), 1.6)); p.drawLine(QPointF(-2 * r, dy), QPointF(2 * r, dy))
        p.setFont(QFont("DejaVu Sans", 7))
        for deg in (-30, -20, -10, 10, 20, 30):
            y = dy - deg * k
            half = r * (0.36 if deg % 20 == 0 else 0.22)
            p.setPen(QPen(QColor("#ffffffcc"), 1)); p.drawLine(QPointF(-half, y), QPointF(half, y))
            p.drawText(QPointF(half + 4, y + 3), str(abs(deg)))
        p.restore()
        p.setPen(QPen(QColor("#ffffff"), 2)); p.setBrush(Qt.BrushStyle.NoBrush); p.drawEllipse(QPointF(cx, cy), r, r)
        p.setPen(QPen(WARN, 3))                                   # fixed aircraft symbol
        p.drawLine(QPointF(cx - r * 0.45, cy), QPointF(cx - r * 0.12, cy)); p.drawLine(QPointF(cx + r * 0.12, cy), QPointF(cx + r * 0.45, cy))
        p.drawLine(QPointF(cx, cy - 2), QPointF(cx, cy + 2))
        p.setPen(TEXT); p.setFont(QFont("DejaVu Sans", 9, QFont.Weight.Bold))
        txt = f"P {self.pitch:+.1f}   R {self.roll:+.1f}   Y {self.yaw:+.0f}" if self.valid else "no attitude"
        p.drawText(QRectF(0, h - 20, w, 18), Qt.AlignmentFlag.AlignCenter, txt)


class StripChart(QWidget):
    """Time series for up to a few series, last `window_s` seconds. Data: {name: [(t, value), ...]}."""

    def __init__(self, title: str, series: list[tuple[str, str, QColor]], unit: str = "", ymin=None, ymax=None, window_s: float = 60.0, parent=None):
        super().__init__(parent)
        self.title, self.series, self.unit = title, series, unit
        self.ymin, self.ymax, self.window_s = ymin, ymax, window_s
        self.data: dict[str, list] = {}
        self.setMinimumHeight(96)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

    def set_data(self, data: dict[str, list]) -> None:
        self.data = data
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        p.fillRect(self.rect(), PANEL)
        left, right, top, bottom = 38, 8, 20, 8
        plot = QRectF(left, top, w - left - right, h - top - bottom)
        pts_all = [pt for key, _, _ in self.series for pt in self.data.get(key, [])]
        tmax = max((pt[0] for pt in pts_all), default=self.window_s)
        first = min((pt[0] for pt in pts_all), default=tmax)
        span = max(10.0, min(self.window_s, tmax - first))        # fill the plot with the data we have, up to window_s
        tmin = tmax - span
        vals = [pt[1] for pt in pts_all if pt[0] >= tmin and math.isfinite(pt[1])]
        lo = self.ymin if self.ymin is not None else (min(vals) if vals else 0.0)
        hi = self.ymax if self.ymax is not None else (max(vals) if vals else 1.0)
        if self.ymin is None and self.ymax is None and hi - lo < 1e-6:
            lo, hi = lo - 1, hi + 1
        elif self.ymin is None or self.ymax is None:
            pad = (hi - lo) * 0.1 or 1.0
            lo = lo if self.ymin is not None else lo - pad
            hi = hi if self.ymax is not None else hi + pad
        p.setPen(QPen(GRID, 1)); p.setFont(QFont("DejaVu Sans", 7))
        for i in range(5):
            y = plot.top() + plot.height() * i / 4
            p.drawLine(QPointF(plot.left(), y), QPointF(plot.right(), y))
            p.setPen(DIM); p.drawText(QRectF(0, y - 7, left - 4, 14), Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter, f"{hi - (hi - lo) * i / 4:.4g}")
            p.setPen(QPen(GRID, 1))
        p.setPen(QPen(GRID, 1)); p.drawRect(plot)
        x = left
        for key, label, col in self.series:
            pts = [pt for pt in self.data.get(key, []) if pt[0] >= tmin and math.isfinite(pt[1])]
            if len(pts) >= 2:
                path = QPainterPath()
                for i, (t, v) in enumerate(pts):
                    px = plot.left() + (t - tmin) / span * plot.width()
                    py = plot.bottom() - (min(max(v, lo), hi) - lo) / (hi - lo) * plot.height()
                    path.moveTo(px, py) if i == 0 else path.lineTo(px, py)
                p.setPen(QPen(col, 1.6)); p.setBrush(Qt.BrushStyle.NoBrush); p.drawPath(path)
            last = f"{pts[-1][1]:.4g}" if pts else "--"
            p.setPen(col); p.setFont(QFont("DejaVu Sans", 8, QFont.Weight.Bold))
            text = f"{label} {last}{self.unit}"
            p.drawText(QPointF(x, 13), text)
            x += p.fontMetrics().horizontalAdvance(text) + 14
        p.setPen(DIM); p.setFont(QFont("DejaVu Sans", 8))
        if x + p.fontMetrics().horizontalAdvance(self.title) + 12 <= w - 6:          # the title only when it fits next to the legend (never on top of it)
            p.drawText(QRectF(0, 0, w - 6, 15), Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter, self.title)
        self.setToolTip(self.title)


class ChannelBars(QWidget):
    NAMES = ("Roll", "Pitch", "Throttle", "Yaw", "AUX1 arm", "AUX2 mode", "AUX3 turtle", "AUX4")

    def __init__(self, parent=None):
        super().__init__(parent)
        self.values = [1500] * 8
        self.setMinimumWidth(230); self.setFixedHeight(8 * 20 + 6)      # fixed: a squeezed layout used to draw the rows over the text below

    def set_values(self, values: list[int]) -> None:
        self.values = values[:8]
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), PANEL)
        label_w, val_w = 82, 38
        bar_x, bar_w = label_w + 4, self.width() - label_w - val_w - 14
        p.setFont(QFont("DejaVu Sans", 8))
        for i, v in enumerate(self.values):
            y = 5 + i * 20
            p.setPen(DIM); p.drawText(QRectF(4, y, label_w, 16), Qt.AlignmentFlag.AlignVCenter, self.NAMES[i])
            p.setPen(QPen(GRID, 1)); p.setBrush(QBrush(BG)); p.drawRoundedRect(QRectF(bar_x, y + 2, bar_w, 12), 3, 3)
            frac = min(max((v - 988) / 1024.0, 0.0), 1.0)
            hot = i == 4 and v > 1700                            # arm channel high = highlight in red
            p.setPen(Qt.PenStyle.NoPen); p.setBrush(QBrush(BAD if hot else ACCENT)); p.drawRoundedRect(QRectF(bar_x, y + 2, max(bar_w * frac, 2), 12), 3, 3)
            p.setPen(TEXT); p.drawText(QRectF(bar_x + bar_w + 4, y, val_w, 16), Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignRight, str(v))


class Led(QWidget):
    """A coloured dot with a text label."""

    def __init__(self, text: str, parent=None):
        super().__init__(parent)
        self.text, self.color, self.detail = text, DIM, ""
        self.setMinimumHeight(22)
        self.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)

    def set(self, color: QColor, detail: str = "") -> None:
        self.color, self.detail = color, detail
        self.update()

    def sizeHint(self):
        from PyQt6.QtCore import QSize
        return QSize(150, 22)

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.setBrush(QBrush(self.color)); p.setPen(Qt.PenStyle.NoPen); p.drawEllipse(QPointF(10, self.height() / 2), 5, 5)
        p.setPen(TEXT); p.setFont(QFont("DejaVu Sans", 9, QFont.Weight.Bold))
        p.drawText(QRectF(22, 0, self.width() - 22, self.height()), Qt.AlignmentFlag.AlignVCenter, self.text + (f"  {self.detail}" if self.detail else ""))


class FlightView(QWidget):
    """Where the drone is: SIDE view (height, hover target, ceiling limit, tilt) and TOP view (x/y trail). In Sim the data is the simulator's
    ground truth; on the live link only the height estimate exists (there is no position source yet), so the top view says so."""
    CEIL_M = 2.5

    def __init__(self, parent=None):
        super().__init__(parent)
        self.z = 0.0
        self.x: Optional[float] = None
        self.y: Optional[float] = None
        self.z_ref: Optional[float] = None
        self.pitch_deg = self.roll_deg = 0.0
        self.armed = False
        self.trail: list = []
        self.label = "no data"
        self.truth = False
        self.setMinimumSize(340, 250)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

    def set_state(self, z, x, y, z_ref, pitch_deg, roll_deg, armed, trail, label, truth) -> None:
        self.z, self.x, self.y, self.z_ref = z or 0.0, x, y, z_ref
        self.pitch_deg, self.roll_deg, self.armed, self.trail, self.label, self.truth = pitch_deg or 0.0, roll_deg or 0.0, armed, trail, label, truth
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), PANEL)
        W, H = self.width(), self.height()
        half = W // 2
        # ---- side view ----
        top, bot = 34, H - 22
        zmax = 3.0
        def ypix(z): return bot - (bot - top) * min(max(z, 0.0), zmax) / zmax
        p.setPen(QPen(GRID, 1))
        for zz in (0.5, 1.0, 1.5, 2.0, 2.5, 3.0):
            p.drawLine(8, int(ypix(zz)), half - 12, int(ypix(zz)))
            p.setPen(DIM); p.setFont(QFont("DejaVu Sans", 7)); p.drawText(QRectF(8, ypix(zz) - 12, 40, 11), f"{zz:.1f} m"); p.setPen(QPen(GRID, 1))
        p.setPen(QPen(BAD, 1, Qt.PenStyle.DashLine)); p.drawLine(8, int(ypix(self.CEIL_M)), half - 12, int(ypix(self.CEIL_M)))
        p.setPen(QPen(DIM, 2)); p.drawLine(8, bot, half - 12, bot)
        if self.z_ref:
            p.setPen(QPen(GOOD, 1, Qt.PenStyle.DashLine)); p.drawLine(8, int(ypix(self.z_ref)), half - 12, int(ypix(self.z_ref)))
        cx, cy = (half - 4) / 2 + 20, ypix(self.z)
        p.save(); p.translate(cx, cy - 6); p.rotate(-self.roll_deg)
        p.setPen(Qt.PenStyle.NoPen); p.setBrush(QBrush(WARN if self.armed else DIM)); p.drawRoundedRect(QRectF(-24, -3, 48, 7), 3, 3)
        p.setBrush(QBrush(TEXT)); p.drawEllipse(QPointF(-24, -6), 5, 3); p.drawEllipse(QPointF(24, -6), 5, 3)     # rotors
        p.restore()
        p.setPen(TEXT); p.setFont(QFont("DejaVu Sans", 9, QFont.Weight.Bold))
        p.setFont(QFont("DejaVu Sans", 8, QFont.Weight.Bold))
        p.drawText(QRectF(8, 2, half - 16, 18), f"SIDE  {self.z:.2f} m" + (f" / target {self.z_ref:.2f}" if self.z_ref else ""))
        # ---- top view ----
        x0, w = half + 4, W - half - 12
        side = min(w, bot - top)
        ox, oy = x0 + (w - side) / 2, top + ((bot - top) - side) / 2
        p.fillRect(QRectF(ox, oy, side, side), BG)
        p.setPen(QPen(GRID, 1))
        span = 3.0                                        # +-1.5 m
        for k in range(7):
            f = k / 6.0
            p.drawLine(QPointF(ox + side * f, oy), QPointF(ox + side * f, oy + side)); p.drawLine(QPointF(ox, oy + side * f), QPointF(ox + side, oy + side * f))
        def pt(x, y): return QPointF(ox + side * (0.5 + y / span), oy + side * (0.5 - x / span))     # x forward = up, y right = right
        p.setPen(QPen(DIM, 1)); p.drawLine(pt(-0.1, 0), pt(0.1, 0)); p.drawLine(pt(0, -0.1), pt(0, 0.1))
        p.setFont(QFont("DejaVu Sans", 9, QFont.Weight.Bold)); p.setPen(TEXT)
        if self.x is None:
            p.setFont(QFont("DejaVu Sans", 8, QFont.Weight.Bold)); p.drawText(QRectF(x0, 2, w, 18), "TOP  no position source (live)")
        else:
            p.setFont(QFont("DejaVu Sans", 8, QFont.Weight.Bold)); p.drawText(QRectF(x0, 2, w, 18), f"TOP  x {self.x:+.2f}  y {self.y:+.2f} m")
            if len(self.trail) > 1:
                p.setPen(QPen(ACCENT, 1.5))
                for a, b in zip(self.trail, self.trail[1:]):
                    p.drawLine(pt(*a), pt(*b))
            p.setPen(Qt.PenStyle.NoPen); p.setBrush(QBrush(WARN if self.armed else DIM)); p.drawEllipse(pt(self.x, self.y), 6, 6)
        p.setPen(DIM); p.setFont(QFont("DejaVu Sans", 8)); p.drawText(QRectF(8, H - 16, W - 16, 14), self.label)
