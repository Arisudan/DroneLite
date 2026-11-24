"""Rectify the real O4 picture to the pinhole profile the Nimbus depth model expects (Air75 profile: 640x480, fx 292.3 px, centred).
Qt-free, ROS-free. Needs the camera's intrinsics K and distortion; without a calibration it assumes an ideal lens of the nominal diagonal FOV (rectilinear for the standard O4, equidistant fisheye for the O4 Pro / Wide).
The O4 lens is much wider than the profile (about 96 deg horizontal), so this is an undistort + centre crop: the outer part of the picture is discarded."""
from __future__ import annotations

import math

import cv2
import numpy as np

PROFILE = {"width": 640, "height": 480, "fx": 292.2957, "fy": 292.2957}


def nominal_K(width: int, height: int, diagonal_fov_deg: float, lens: str = "rectilinear") -> np.ndarray:
    """Intrinsics of an ideal lens with the given diagonal FOV: rectilinear (r = f tan t) or equidistant fisheye (r = f t; the O4 Pro / Wide are near-fisheye)."""
    half = math.hypot(width, height) / 2.0; t = math.radians(diagonal_fov_deg) / 2.0
    f = half / math.tan(t) if lens == "rectilinear" else half / t
    return np.array([[f, 0, width / 2.0], [0, f, height / 2.0], [0, 0, 1.0]])


class Rectifier:
    def __init__(self, width: int, height: int, K=None, dist=None, diagonal_fov_deg: float = 117.6, profile: dict | None = None, lens: str = "rectilinear") -> None:
        pr = profile or PROFILE; self.size = (pr["width"], pr["height"])
        self.K = np.array(K, float) if K is not None else nominal_K(width, height, diagonal_fov_deg, lens)
        self.fisheye = lens == "equidistant"; self.dist = np.zeros(4 if self.fisheye else 5) if dist is None else np.array(dist, float)
        newK = np.array([[pr["fx"], 0, pr["width"] / 2.0], [0, pr["fy"], pr["height"] / 2.0], [0, 0, 1.0]])
        if self.fisheye: self.m1, self.m2 = cv2.fisheye.initUndistortRectifyMap(self.K, self.dist[:4], np.eye(3), newK, self.size, cv2.CV_16SC2)
        else: self.m1, self.m2 = cv2.initUndistortRectifyMap(self.K, self.dist, np.eye(3), newK, self.size, cv2.CV_16SC2)

    def __call__(self, frame: np.ndarray) -> np.ndarray:
        return cv2.remap(frame, self.m1, self.m2, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
