"""Lens calibration of the real O4 from its live CosmoStreamer picture (Video page, 'Calibrate lens'). Qt-free.

The picture the GCS receives (1280x720, a 16:9 crop of the sensor) is the one the map rectifies, so it is calibrated as it arrives. A checkerboard (shown full screen
by the GCS on this laptop, or printed: tools_scripts/o4_calib/checkerboard_9x6.png) is held in view; a view is taken only when the board is found, the camera is
still (no motion blur) and the view differs from those already taken (position, size, tilt). Then OpenCV's fisheye model (the O4 Pro is near-fisheye, equidistant;
config variant 'pro') or the pinhole model (standard O4) is solved and written into gazebo/config/o4_camera.yaml `calibration:` (the old file is archived first).
o4_map_node.py --rectify reads it on its next start."""
from __future__ import annotations

import datetime
import os
import re
import shutil
import threading
import time
from typing import List, Optional, Tuple

import cv2
import numpy as np

DF = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
CONFIG = os.path.join(DF, "gazebo", "config", "o4_camera.yaml")
ARCHIVE = os.path.join(DF, "archive", "o4_camera_yaml")
BOARDS = [(9, 6), (8, 6), (7, 5), (10, 7), (6, 4), (11, 8), (7, 6), (8, 5)]      # inner corners; the first found is kept
TARGET_VIEWS = 25; MIN_VIEWS = 12; STILL_PX = 2.0; DIVERSE = 0.08; GOOD_RMS = 1.0; MAX_RMS = 1.5


def config_model(path: str = CONFIG) -> str:
    """'fisheye' for an equidistant lens variant (O4 Pro / Wide), else 'pinhole'."""
    try:
        import yaml
        cfg = yaml.safe_load(open(path)); lens = cfg["variants"][cfg["variant"]].get("lens", "rectilinear")
    except Exception:                                                     # noqa: BLE001
        lens = "equidistant"
    return "fisheye" if lens == "equidistant" else "pinhole"


def board_image(cols: int = 9, rows: int = 6, square: int = 90, margin: int = 90) -> np.ndarray:
    """A checkerboard with cols x rows INNER corners (so (cols+1) x (rows+1) squares), white margin, grey-scale uint8."""
    w, h = (cols + 1) * square + 2 * margin, (rows + 1) * square + 2 * margin; img = np.full((h, w), 255, np.uint8)
    for r in range(rows + 1):
        for c in range(cols + 1):
            if (r + c) % 2 == 0: img[margin + r * square:margin + (r + 1) * square, margin + c * square:margin + (c + 1) * square] = 0
    return img


def find_board(gray: np.ndarray, pattern: Optional[Tuple[int, int]] = None) -> Tuple[Optional[Tuple[int, int]], Optional[np.ndarray]]:
    """(pattern, corners N x 2 float32 in full-resolution pixels) or (None, None). Searched on a <= 640 px copy (fast), refined at full resolution."""
    s = min(1.0, 640.0 / gray.shape[1]); small = cv2.resize(gray, None, fx=s, fy=s, interpolation=cv2.INTER_AREA) if s < 1 else gray
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK
    for pat in ([pattern] if pattern else BOARDS):
        ok, c = cv2.findChessboardCorners(small, pat, flags=flags)
        if ok:
            c = (c.reshape(-1, 2) / s).astype(np.float32).reshape(-1, 1, 2)
            c = cv2.cornerSubPix(gray, c, (7, 7), (-1, -1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01))
            return pat, c.reshape(-1, 2)
    return None, None


def _describe(c: np.ndarray, pat: Tuple[int, int], size: Tuple[int, int]) -> np.ndarray:
    """Position (centre), size and tilt of a board view, each about 0..1."""
    W, H = size; cols, rows = pat; g = c.reshape(rows, cols, 2)
    top, bot = np.linalg.norm(g[0, -1] - g[0, 0]), np.linalg.norm(g[-1, -1] - g[-1, 0]); lef, rig = np.linalg.norm(g[-1, 0] - g[0, 0]), np.linalg.norm(g[-1, -1] - g[0, -1])
    area = cv2.contourArea(np.array([g[0, 0], g[0, -1], g[-1, -1], g[-1, 0]], np.float32))
    return np.array([c[:, 0].mean() / W, c[:, 1].mean() / H, np.sqrt(area / (W * H)), 2 * (top - bot) / (top + bot), 2 * (lef - rig) / (lef + rig)])


def _fe(name: str) -> int:
    """A fisheye calibration flag: OpenCV 4 keeps it in cv2.fisheye, OpenCV 5 moved it to the top level (cv2.CALIB_*)."""
    return getattr(cv2.fisheye, name, None) if hasattr(cv2.fisheye, name) else getattr(cv2, name)


def solve(views: List[np.ndarray], pat: Tuple[int, int], size: Tuple[int, int], model: str = "fisheye"):
    """Returns dict(model, size, fx, fy, cx, cy, dist, rms_px, views)."""
    cols, rows = pat; obj = np.zeros((cols * rows, 3), np.float64); obj[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    W, H = size; crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-7)
    if model == "fisheye":
        O = [obj.reshape(1, -1, 3) for _ in views]; I = [v.astype(np.float64).reshape(1, -1, 2) for v in views]
        K0 = np.array([[W / 2.5, 0, W / 2.0], [0, W / 2.5, H / 2.0], [0, 0, 1]], np.float64); D0 = np.zeros((4, 1))
        base = _fe("CALIB_RECOMPUTE_EXTRINSIC") | _fe("CALIB_FIX_SKEW")
        try:
            rms, K, D, _r, _t = cv2.fisheye.calibrate(O, I, (W, H), K0, D0, flags=base | _fe("CALIB_CHECK_COND"), criteria=crit)
        except cv2.error:                                                 # an ill-conditioned view: solve without the check
            rms, K, D, _r, _t = cv2.fisheye.calibrate(O, I, (W, H), K0, D0, flags=base, criteria=crit)
        dist = [float(x) for x in np.ravel(D)[:4]]
    else:
        rms, K, D, _r, _t = cv2.calibrateCamera([obj.astype(np.float32)] * len(views), [v.reshape(-1, 1, 2) for v in views], (W, H), None, None, criteria=crit)
        dist = [float(x) for x in np.ravel(D)[:5]]
    return {"model": model, "size": [int(W), int(H)], "fx": float(K[0, 0]), "fy": float(K[1, 1]), "cx": float(K[0, 2]), "cy": float(K[1, 2]), "dist": dist,
            "rms_px": float(rms), "views": len(views)}


def save(cal: dict, path: str = CONFIG) -> str:
    """Writes `calibration:` in o4_camera.yaml (comments kept); the file before the change is archived first. Returns the archive path."""
    os.makedirs(ARCHIVE, exist_ok=True); stamp = datetime.datetime.now().strftime("%Y-%m-%d_%H%M%S")
    arch = os.path.join(ARCHIVE, f"o4_camera_{stamp}.yaml"); shutil.copy2(path, arch)
    r = lambda x: round(x, 4)
    line = (f"calibration: {{model: {cal['model']}, size: [{cal['size'][0]}, {cal['size'][1]}], fx: {r(cal['fx'])}, fy: {r(cal['fy'])}, cx: {r(cal['cx'])}, cy: {r(cal['cy'])}, "
            f"dist: [{', '.join(f'{d:.6g}' for d in cal['dist'])}], rms_px: {r(cal['rms_px'])}, views: {cal['views']}, date: '{stamp[:10]}'}}"
            f"   # measured with the GCS (Video page, Calibrate lens)")
    s = open(path).read(); s2, n = re.subn(r"^calibration:.*$", line, s, count=1, flags=re.M)
    if n == 0: s2 = s.rstrip("\n") + "\n" + line + "\n"
    open(path, "w").write(s2)
    return arch


class LensCalibrator:
    """Runs in its own thread on the newest video frame (5 Hz), so the picture never waits for it. All state is read by the GUI for the status line."""

    def __init__(self, get_frame, model: str = "fisheye", config: str = CONFIG) -> None:
        self.get_frame = get_frame; self.model = model; self.config = config
        self.pattern = None; self.views: List[np.ndarray] = []; self.descs: List[np.ndarray] = []; self.size = None
        self.corners = None; self.t_corners = 0.0; self._prev = None; self.hint = "Show the checkerboard to the camera"; self.result = None; self.saved = ""
        self.state = "collecting"; self.cover = np.zeros((3, 3), bool); self._stop = threading.Event(); self._lock = threading.Lock()
        threading.Thread(target=self._run, name="lens-calib", daemon=True).start()

    def _run(self) -> None:
        last_t = None
        while not self._stop.is_set() and self.state == "collecting":
            fr = self.get_frame(); f, t = fr if isinstance(fr, tuple) else (fr, time.time())
            if f is None or t == last_t: time.sleep(0.05); continue
            last_t = t; self.feed(f); time.sleep(0.15)
            if len(self.views) >= TARGET_VIEWS: self.finish()

    def feed(self, bgr: np.ndarray) -> bool:
        """One frame; True when it was taken as a view."""
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY) if bgr.ndim == 3 else bgr; H, W = gray.shape
        if self.size is not None and self.size != (W, H): self.hint = "The picture size changed: calibration restarted"; self.reset()
        self.size = (W, H); pat, c = find_board(gray, self.pattern)
        if c is None:
            self._prev = None; self.hint = "Board not found: the whole board in view, sharp, not too far"; return False
        self.pattern = pat; self.corners = c; self.t_corners = time.time()
        still = self._prev is not None and float(np.mean(np.linalg.norm(c - self._prev, axis=1))) < STILL_PX; self._prev = c
        if not still: self.hint = "Board found: hold still"; return False
        d = _describe(c, pat, self.size)
        if self.descs and min(float(np.linalg.norm(d - e)) for e in self.descs) < DIVERSE:
            self.hint = "Move to a new spot: " + self._next_hint(); return False
        with self._lock:
            self.views.append(c.copy()); self.descs.append(d)
            for x, y in c[:: max(1, len(c) // 12)]: self.cover[min(2, int(3 * y / H)), min(2, int(3 * x / W))] = True
        self.hint = "View taken. Next: " + self._next_hint(); return True

    def _next_hint(self) -> str:
        empty = [(r, c) for r in range(3) for c in range(3) if not self.cover[r, c]]
        if empty:
            r, c = empty[0]; return f"the board in the {['top', 'middle', 'bottom'][r]} {['left', 'centre', 'right'][c]} of the picture"
        n = len(self.views)
        return ["tilt the board left / right", "tilt the board up / down", "nearer (fill the picture)", "further away"][n % 4]

    def finish(self) -> None:
        """Solve with the views taken (needs MIN_VIEWS), save when good. Sets state to done / failed."""
        if self.state != "collecting": return
        self._stop.set()
        if len(self.views) < MIN_VIEWS:
            self.state = "failed"; self.hint = f"Stopped with {len(self.views)} views: at least {MIN_VIEWS} are needed. Nothing saved."; return
        self.state = "solving"; self.hint = "Solving the lens model"
        try:
            with self._lock: views = list(self.views)
            self.result = solve(views, self.pattern, self.size, self.model)
        except Exception as e:                                            # noqa: BLE001
            self.state = "failed"; self.hint = f"The solve failed ({e}). Nothing saved: take more varied views."; return
        rms = self.result["rms_px"]
        if rms > MAX_RMS:
            self.state = "failed"; self.hint = f"Not saved: the fit error is {rms:.2f} px (needs <= {MAX_RMS}): keep the board flat and still, more varied views."; return
        self.saved = save(self.result, self.config); self.state = "done"
        self.hint = (f"Lens calibrated: fit error {rms:.2f} px{'' if rms <= GOOD_RMS else ' (usable, not great)'}, fx {self.result['fx']:.1f} px, "
                     f"{self.result['views']} views. Saved in gazebo/config/o4_camera.yaml; Stop map + Start map to use it.")

    def reset(self) -> None:
        with self._lock: self.views, self.descs, self.pattern, self._prev = [], [], None, None; self.cover[:] = False

    def stop(self) -> None:
        self._stop.set()
        if self.state == "collecting": self.state = "stopped"

    def overlay(self, bgr: np.ndarray) -> np.ndarray:
        """The found corners drawn on a copy of the picture (only while calibrating, only when fresh)."""
        if self.corners is None or self.pattern is None or time.time() - self.t_corners > 0.5: return bgr
        out = bgr.copy(); cv2.drawChessboardCorners(out, self.pattern, self.corners.reshape(-1, 1, 2), True); return out
