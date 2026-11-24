"""2D RTAB-Map sparse map view (the Flop GCS map, in this GCS's look). Layers, bottom to top: never-seen = page colour, seen free space = panel colour,
RTAB-Map's raw obstacle cells = faint, the 1-pixel wall skeleton (/map_thin) = accent colour, the drone's trail and the drone (a triangle pointing the way it
faces). North is up (ROS map frame: x east, y north). A scale bar shows a round distance in metres. The picture is rebuilt only when a new grid arrives.
Reads only; nothing here sends a command."""
from __future__ import annotations

import math
from collections import deque

import cv2
import numpy as np
from PyQt6.QtCore import QPointF, QRectF, Qt
from PyQt6.QtGui import QColor, QImage, QPainter, QPen, QPolygonF
from PyQt6.QtWidgets import QSizePolicy, QWidget

from . import theme

MARGIN_CELLS = 12
TRAIL_STEP_M = 0.05
_ROUND_M = (0.1, 0.2, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0)


def _rgb(c: str) -> tuple[int, int, int]:
    q = QColor(c); return q.red(), q.green(), q.blue()


class SlamMapView(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.setMinimumHeight(260); self.setMinimumWidth(220); self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.text = "Map off"; self._key = None; self._img: QImage | None = None; self._geo = None      # geo = (ox, oy, res, cx0, cy1) of the drawn crop
        self.pose = None; self.trail: deque = deque(maxlen=1500); self._wall = None; self._sub = None

    # ---- data in
    def clear(self, text: str = "Map cleared") -> None:
        self._key = None; self._img = None; self._geo = None; self.pose = None; self.trail.clear(); self.text = text; self.update()

    def set_idle(self, text: str) -> None:
        if self._img is not None or self.text != text:
            self._img = None; self._geo = None; self._key = None; self.text = text; self.update()

    def set_data(self, raw, thin, pose, text: str) -> None:
        """raw / thin: dicts from MapClient ({"grid","res","ox","oy","t"}) or None; pose (x, y, yaw) in the map frame or None."""
        key = (raw["t"] if raw else 0.0, thin["t"] if thin else 0.0)
        if key != self._key:
            self._key = key; self._build(raw, thin)
        self.pose = pose; self.text = text
        if pose is not None and (not self.trail or math.hypot(pose[0] - self.trail[-1][0], pose[1] - self.trail[-1][1]) > TRAIL_STEP_M):
            self.trail.append((pose[0], pose[1]))
        self.update()

    def _build(self, raw, thin) -> None:
        base = raw or thin
        if base is None:
            self._img = None; self._geo = None; return
        g = base["grid"]; H, W = g.shape; res, ox, oy = base["res"], base["ox"], base["oy"]
        known = g >= 0
        if thin is not None and thin["grid"].shape == g.shape:
            known = known | (thin["grid"] == 100)
        ys, xs = np.nonzero(known)
        if len(ys) == 0:
            self._img = None; self._geo = None; return
        cx0, cx1 = max(0, xs.min() - MARGIN_CELLS), min(W, xs.max() + 1 + MARGIN_CELLS); cy0, cy1 = max(0, ys.min() - MARGIN_CELLS), min(H, ys.max() + 1 + MARGIN_CELLS)
        rgba = np.zeros((cy1 - cy0, cx1 - cx0, 4), np.uint8)
        sub = g[cy0:cy1, cx0:cx1]
        rgba[sub >= 0] = (*_rgb(theme.PANEL), 255)
        rgba[sub >= 65] = (*_rgb(theme.HAIR2), 255)
        if thin is not None and thin["grid"].shape == g.shape:
            wall = (thin["grid"][cy0:cy1, cx0:cx1] == 100).astype(np.uint8)
            rgba[wall > 0] = (*_rgb(theme.ACC), 255)
        rgba = np.ascontiguousarray(rgba[::-1])                       # row 0 of the grid is the south edge
        self._img = QImage(rgba.data, rgba.shape[1], rgba.shape[0], 4 * rgba.shape[1], QImage.Format.Format_RGBA8888).copy()
        self._geo = (ox, oy, res, cx0, cy1)
        self._wall = wall[::-1].copy() if thin is not None and thin["grid"].shape == g.shape else None
        self._sub = rgba

    # ---- drawing
    def _fit(self):
        ih, iw = self._img.height(), self._img.width()
        s = min((self.width() - 16) / iw, (self.height() - 16) / ih)
        return s, (self.width() - iw * s) / 2, (self.height() - ih * s) / 2

    def _to_px(self, x: float, y: float, fit):
        ox, oy, res, cx0, cy1 = self._geo; s, x0, y0 = fit
        return QPointF(x0 + ((x - ox) / res - cx0) * s, y0 + (cy1 - (y - oy) / res) * s)

    def paintEvent(self, _e) -> None:
        p = QPainter(self); p.fillRect(self.rect(), QColor(theme.BG))
        if self._img is None or self._geo is None:
            p.setPen(QColor(theme.DIM)); p.drawText(self.rect().adjusted(10, 8, -10, -8), Qt.AlignmentFlag.AlignCenter, self.text); return
        s, x0, y0 = fit = self._fit()
        iw, ih = self._img.width(), self._img.height()
        if s < 1.0 and self._wall is not None:                       # zoomed out: thicken the 1-cell walls so they stay visible instead of fading
            k = int(math.ceil(1.0 / s)); img = self._sub.copy(); w = cv2.dilate(self._wall, np.ones((k, k), np.uint8)) > 0
            img[w] = (*_rgb(theme.ACC), 255); shown = QImage(img.data, iw, ih, 4 * iw, QImage.Format.Format_RGBA8888).copy()
        else:
            shown = self._img
        p.setRenderHint(QPainter.RenderHint.SmoothPixmapTransform, s < 1.0)
        p.drawImage(QRectF(x0, y0, iw * s, ih * s), shown)
        if len(self.trail) > 1:
            p.setPen(QPen(QColor(theme.TXT), 1.4, Qt.PenStyle.DotLine)); pts = [self._to_px(a, b, fit) for a, b in self.trail]
            for a, b in zip(pts, pts[1:]): p.drawLine(a, b)
        if self.pose is not None:
            c = self._to_px(self.pose[0], self.pose[1], fit); yaw = self.pose[2]; r = 12
            tip = QPointF(c.x() + r * math.cos(yaw), c.y() - r * math.sin(yaw))
            l = QPointF(c.x() + r * 0.7 * math.cos(yaw + 2.5), c.y() - r * 0.7 * math.sin(yaw + 2.5)); rr = QPointF(c.x() + r * 0.7 * math.cos(yaw - 2.5), c.y() - r * 0.7 * math.sin(yaw - 2.5))
            p.setPen(QPen(QColor(theme.BG), 1.5)); p.setBrush(QColor(theme.TXT)); p.drawPolygon(QPolygonF([tip, l, rr]))
        # scale bar: the longest round distance that fits in a quarter of the picture
        m_per_px = self._geo[2] / s; target = (self.width() / 4) * m_per_px; bar = max((d for d in _ROUND_M if d <= target), default=_ROUND_M[0])
        L = bar / m_per_px; bx, by = 14, self.height() - 34
        p.setPen(QPen(QColor(theme.TXT), 2)); p.drawLine(QPointF(bx, by), QPointF(bx + L, by)); p.drawLine(QPointF(bx, by - 4), QPointF(bx, by + 4)); p.drawLine(QPointF(bx + L, by - 4), QPointF(bx + L, by + 4))
        p.setPen(QColor(theme.TXT)); p.drawText(QPointF(bx + L + 8, by + 4), f"{bar:g} m")
        p.setPen(QColor(theme.DIM)); p.drawText(self.rect().adjusted(10, 8, -10, -8), Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignBottom, self.text)
