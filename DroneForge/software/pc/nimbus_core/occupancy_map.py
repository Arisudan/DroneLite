"""Accumulated sparse 2D occupancy map built from the Nimbus live voxel floor plan (the idea of the Flop project's /map and /map_thin, with the drone's own pose).

Qt-free and ROS-free: used by the Gazebo explorer and by the GCS map panel. Map frame = start point (0, 0), x east, y north, yaw counter-clockwise from x.
Each depth frame gives a sparse top-down view in the camera frame (cells[ix, iz, count]; ix 0 = left, iz forward). With the pose at the time the frame was
taken we (a) add the cells as hits, (b) carve free space along each column from the camera to its first hit, in a log-odds grid.
The pose must come from the drone (FC EKF position + attitude); it drifts on a real drone, so this is a local-quality map, not a surveyed one.
"""
from __future__ import annotations

import math

import cv2
import numpy as np


class OccupancyMap:
    def __init__(self, res: float = 0.1, size_m: float = 40.0, max_range: float = 4.5, free_range: float = 2.5) -> None:
        self.res = res; self.n = int(size_m / res); self.c = self.n // 2
        self.L = np.zeros((self.n, self.n), np.float32)            # log-odds: > 0 occupied, < 0 free, 0 never seen
        self.max_range = max_range; self.free_range = free_range; self.frames = 0
        self.trail: list[tuple[float, float]] = []

    # -- coordinates
    def to_cell(self, x: float, y: float) -> tuple[int, int]:
        return int(round(x / self.res)) + self.c, int(round(y / self.res)) + self.c      # (col, row)

    def to_xy(self, col: float, row: float) -> tuple[float, float]:
        return (col - self.c) * self.res, (row - self.c) * self.res

    # -- update
    def update(self, fp: dict, x: float, y: float, yaw: float) -> None:
        vox, nx = fp["vox"], fp["nx"]; xhalf = nx * vox / 2.0
        cy, sy = math.cos(yaw), math.sin(yaw); hit: dict[int, int] = {}
        for ix, iz, cnt in fp["cells"]:
            if (iz + 0.5) * vox <= self.max_range: hit[ix] = min(hit.get(ix, 999), iz)
        L = self.L
        for ix in range(nx):
            lat = xhalf - (ix + 0.5) * vox                        # metres to the LEFT of the optical axis
            reach = (hit[ix] * vox - 0.05) if ix in hit else self.free_range
            if reach > 0.15:
                t = np.arange(0.15, reach, self.res * 0.9)
                wx = x + cy * t - sy * lat; wy = y + sy * t + cy * lat
                cc = np.rint(wx / self.res).astype(int) + self.c; rr = np.rint(wy / self.res).astype(int) + self.c
                ok = (cc >= 0) & (cc < self.n) & (rr >= 0) & (rr < self.n)
                L[rr[ok], cc[ok]] = np.maximum(L[rr[ok], cc[ok]] - 0.35, -4.0)
        for ix, iz, cnt in fp["cells"]:
            f = (iz + 0.5) * vox
            if f > self.max_range: continue
            lat = xhalf - (ix + 0.5) * vox
            wx = x + cy * f - sy * lat; wy = y + sy * f + cy * lat
            col, row = self.to_cell(wx, wy)
            if 0 <= col < self.n and 0 <= row < self.n: L[row, col] = min(L[row, col] + 0.7, 4.0)
        self.frames += 1; self.trail.append((x, y))

    def mark_obstacle(self, x: float, y: float, yaw: float, dist: float, width: float = 0.5) -> None:
        """Stamp an obstacle `dist` metres ahead of the pose (the drone was blocked there: a wall seen through a gap, glass, ...)."""
        cy, sy = math.cos(yaw), math.sin(yaw)
        for lat in np.arange(-width / 2, width / 2 + 1e-6, self.res):
            for f in (dist + 0.1, dist + 0.2):
                col, row = self.to_cell(x + cy * f - sy * lat, y + sy * f + cy * lat)
                if 0 <= col < self.n and 0 <= row < self.n: self.L[row, col] = max(self.L[row, col], 3.0)

    # -- classification
    def occupied(self, thin: bool = True, min_area: int = 3) -> np.ndarray:
        occ = (self.L >= 0.6).astype(np.uint8)
        if thin and occ.any():                                   # purge noise blobs (the Flop map_thinning idea): drop isolated groups of fewer than min_area cells
            nlab, lab, stats, _ = cv2.connectedComponentsWithStats(occ, connectivity=8)
            small = np.nonzero(stats[:, cv2.CC_STAT_AREA] < min_area)[0]; small = small[small != 0]
            if len(small): occ[np.isin(lab, small)] = 0
        return occ.astype(bool)

    def free(self) -> np.ndarray:
        return self.L <= -0.5

    def unknown(self) -> np.ndarray:
        return np.abs(self.L) < 0.05

    def bounds(self, margin: int = 15):
        known = ~self.unknown()
        if not known.any(): return self.c - 50, self.c + 50, self.c - 50, self.c + 50
        rows = np.nonzero(known.any(1))[0]; cols = np.nonzero(known.any(0))[0]
        return max(0, cols[0] - margin), min(self.n, cols[-1] + margin), max(0, rows[0] - margin), min(self.n, rows[-1] + margin)

    # -- drawing (greys plus the one soft-blue accent)
    def render(self, pose=None, path=None, goal=None, scale: int = 6, show_trail: bool = True) -> np.ndarray:
        c0, c1, r0, r1 = self.bounds(); sub = self.L[r0:r1, c0:c1]
        img = np.full(sub.shape + (3,), 22, np.uint8)
        img[sub <= -0.5] = (58, 58, 58); img[sub <= -2.0] = (74, 74, 74)
        occ = self.occupied()[r0:r1, c0:c1]; img[occ] = (255, 203, 169)             # BGR of the accent #A9CBFF
        img = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST); img = img[::-1].copy()   # north up
        H = img.shape[0]

        def px(x, y):
            col, row = self.to_cell(x, y); return int((col - c0 + 0.5) * scale), int(H - (row - r0 + 0.5) * scale)
        if show_trail and len(self.trail) > 1:
            pts = np.array([px(*p) for p in self.trail[-600:]], np.int32); cv2.polylines(img, [pts], False, (150, 150, 150), 1, cv2.LINE_AA)
        if path:
            pts = np.array([px(*p) for p in path], np.int32); cv2.polylines(img, [pts], False, (120, 200, 255), 2, cv2.LINE_AA)
        if goal: cv2.circle(img, px(*goal), 6, (120, 200, 255), 2, cv2.LINE_AA)
        if pose:
            x, y, yaw = pose; p = px(x, y); q = px(x + 0.35 * math.cos(yaw), y + 0.35 * math.sin(yaw)); cv2.circle(img, p, 6, (240, 240, 240), -1, cv2.LINE_AA); cv2.line(img, p, q, (240, 240, 240), 2, cv2.LINE_AA)
        cv2.putText(img, f"{self.frames} frames  (blue = obstacle, grey = free, dark = unknown)", (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)
        return img
