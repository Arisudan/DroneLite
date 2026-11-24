"""Occupancy map taken from an external SLAM (RTAB-Map /map, nav_msgs/OccupancyGrid) with the same interface the Roamer uses for its own OccupancyMap.
Qt-free and ROS-free: the caller passes the grid fields. Also the thinning idea of the Flop map_thinning_node: purge noise blobs and reduce thick walls to a
one-cell skeleton for display (OpenCV here has no ximgproc, so a morphological skeleton is used)."""
from __future__ import annotations

import math

import cv2
import numpy as np

from .occupancy_map import OccupancyMap


def skeleton(mask: np.ndarray) -> np.ndarray:
    """Morphological skeleton (thick walls -> centre lines)."""
    img = (mask > 0).astype(np.uint8) * 255; out = np.zeros_like(img); k = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
    while img.any():
        er = cv2.erode(img, k); op = cv2.dilate(er, k); out |= cv2.subtract(img, op); img = er
    return out > 0


class GridMap(OccupancyMap):
    accumulates = False            # the SLAM accumulates; this class only holds its latest grid

    def __init__(self) -> None:
        self.res = 0.05; self.L = np.zeros((1, 1), np.float32); self.ox = 0.0; self.oy = 0.0; self.frames = 0; self.trail: list = []; self.bumps: list = []
        self.n = 1; self.c = 0; self.max_range = 4.5; self.free_range = 2.5

    # -- grid in
    def set_grid(self, width: int, height: int, res: float, ox: float, oy: float, data) -> None:
        p = np.asarray(data, np.int16).reshape(height, width)
        L = np.zeros((height, width), np.float32); known = p >= 0
        L[known] = (p[known].astype(np.float32) - 50.0) / 50.0 * 3.0                  # 0 % -> -3 (free), 100 % -> +3 (occupied), unknown -> 0
        self.L = L; self.res = res; self.ox = ox; self.oy = oy; self.n = max(width, height); self.frames += 1

    # -- coordinates (cell centres; origin is the grid's lower-left corner)
    def to_cell(self, x: float, y: float):
        return int(math.floor((x - self.ox) / self.res)), int(math.floor((y - self.oy) / self.res))

    def to_xy(self, col: float, row: float):
        return self.ox + (col + 0.5) * self.res, self.oy + (row + 0.5) * self.res

    def update(self, *a, **k) -> None:          # not used: the SLAM does the accumulating
        return

    def mark_obstacle(self, x: float, y: float, yaw: float, dist: float, width: float = 1.2) -> None:
        """Remember that the drone was blocked ahead (glass, something the depth cannot see): a wall segment across the heading, kept on top of every new grid."""
        cy, sy = math.cos(yaw), math.sin(yaw); f = dist + 0.15
        self.bumps.append(((x + cy * f + sy * width / 2, y + sy * f - cy * width / 2), (x + cy * f - sy * width / 2, y + sy * f + cy * width / 2)))

    def _bump_mask(self) -> np.ndarray:
        m = np.zeros(self.L.shape, np.uint8)
        for a, b in self.bumps:
            (c1, r1), (c2, r2) = self.to_cell(*a), self.to_cell(*b); cv2.line(m, (c1, r1), (c2, r2), 1, 2)
        return m

    def _with_bumps(self) -> np.ndarray:
        L = self.L
        if self.bumps:
            L = L.copy(); L[self._bump_mask() > 0] = 3.0
        return L

    def occupied(self, thin: bool = True, min_area: int = 4) -> np.ndarray:
        occ = (self._with_bumps() >= 0.6).astype(np.uint8)
        if thin and occ.any():
            nlab, lab, stats, _ = cv2.connectedComponentsWithStats(occ, connectivity=8)
            small = np.nonzero(stats[:, cv2.CC_STAT_AREA] < min_area)[0]; small = small[small != 0]          # noise blobs: monocular depth leaves speckle; walls and furniture are larger
            if len(small): occ[np.isin(lab, small)] = 0
        return occ.astype(bool)

    def free(self) -> np.ndarray:
        return self._with_bumps() <= -0.5

    def unknown(self) -> np.ndarray:
        return np.abs(self._with_bumps()) < 0.05

    def bounds(self, margin: int = 15):
        known = ~self.unknown()
        if not known.any(): return 0, self.L.shape[1], 0, self.L.shape[0]
        rows = np.nonzero(known.any(1))[0]; cols = np.nonzero(known.any(0))[0]
        return max(0, cols[0] - margin), min(self.L.shape[1], cols[-1] + margin), max(0, rows[0] - margin), min(self.L.shape[0], rows[-1] + margin)

    def render(self, pose=None, path=None, goal=None, scale: int = 6, show_trail: bool = True) -> np.ndarray:
        c0, c1, r0, r1 = self.bounds(); L = self._with_bumps()[r0:r1, c0:c1]
        img = np.full(L.shape + (3,), 22, np.uint8); img[L <= -0.5] = (58, 58, 58)
        occ = self.occupied()[r0:r1, c0:c1]; img[skeleton(occ) | (cv2.erode(occ.astype(np.uint8), np.ones((3, 3), np.uint8)) > 0) * False] = (255, 203, 169)   # walls as one-cell lines (the Flop /map_thin look)
        sc = max(2, int(scale * 0.05 / self.res * 1.0)); img = cv2.resize(img, None, fx=sc, fy=sc, interpolation=cv2.INTER_NEAREST)[::-1].copy(); H = img.shape[0]

        def px(x, y):
            col, row = self.to_cell(x, y); return int((col - c0 + 0.5) * sc), int(H - (row - r0 + 0.5) * sc)
        if path: cv2.polylines(img, [np.array([px(*p) for p in path], np.int32)], False, (120, 200, 255), 2, cv2.LINE_AA)
        if goal: cv2.circle(img, px(*goal), 6, (120, 200, 255), 2, cv2.LINE_AA)
        if pose:
            x, y, yaw = pose; p = px(x, y); q = px(x + 0.35 * math.cos(yaw), y + 0.35 * math.sin(yaw)); cv2.circle(img, p, 6, (240, 240, 240), -1, cv2.LINE_AA); cv2.line(img, p, q, (240, 240, 240), 2, cv2.LINE_AA)
        cv2.putText(img, f"RTAB-Map grid, {self.frames} updates  (blue = walls, grey = free, dark = unknown)", (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)
        return img
