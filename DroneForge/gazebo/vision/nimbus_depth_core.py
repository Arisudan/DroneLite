#!/usr/bin/env python3
"""ROS-free core of the Nimbus depth step (shared by nimbus_perception.py on the laptop and depth_server.py on the GPU PC)."""
import os, time
import numpy as np
import cv2
import onnxruntime as ort
try: ort.preload_dlls()                                      # loads the pip CUDA / cuDNN libraries (onnxruntime-gpu[cuda,cudnn]) so CUDA works without LD_LIBRARY_PATH
except Exception: pass
HERE = os.path.dirname(os.path.abspath(__file__)); MODELS = os.path.join(HERE, "models", "nimbus")
MEAN = np.array([0.485, 0.456, 0.406], np.float32); STD = np.array([0.229, 0.224, 0.225], np.float32)
IN_W, IN_H = 504, 336; ROWS, COLS = 6, 8


def decode_msg(msg):
    a = np.frombuffer(msg.data, np.uint8)
    ch = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}.get(msg.encoding, 3)
    a = a.reshape(msg.height, msg.step)[:, :msg.width * ch].reshape(msg.height, msg.width, ch)
    if msg.encoding == "bgr8": a = a[..., ::-1]
    return np.ascontiguousarray(a[..., :3]) if ch >= 3 else np.repeat(a, 3, -1)


class Da3:
    def __init__(self, threads, providers=None):
        so = ort.SessionOptions(); so.intra_op_num_threads = threads; so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        if not providers:                                    # default: the GPU when there is one (same settings as depth_server.py), else the CPU
            providers = [("CUDAExecutionProvider", {"arena_extend_strategy": "kSameAsRequested", "gpu_mem_limit": 8 << 30, "cudnn_conv_algo_search": "HEURISTIC"})] if "CUDAExecutionProvider" in ort.get_available_providers() else []
            providers.append("CPUExecutionProvider")
        self.s = ort.InferenceSession(os.path.join(MODELS, "da3_metric.onnx"), so, providers=providers)
        self.inp = self.s.get_inputs()[0].name
        outs = [o.name for o in self.s.get_outputs()]; self.out = next((o for o in outs if "depth" in o.lower() or "metric" in o.lower()), outs[0])

    def __call__(self, rgb):
        x = cv2.resize(rgb, (IN_W, IN_H), interpolation=cv2.INTER_LINEAR).astype(np.float32) / 255.0
        x = ((x - MEAN) / STD).transpose(2, 0, 1)[None]
        d = self.s.run([self.out], {self.inp: np.ascontiguousarray(x)})[0]
        return np.asarray(d).reshape(-1, d.shape[-2], d.shape[-1])[0]


class FloorScale:
    """Per-frame metric scale for DA3 from the floor (2026-11-21: DA3's floor depth jumps +-13 % frame to frame, median +3 %; every VO key frame and the RTAB-Map
    cloud inherited it). Level-ish camera at a KNOWN height (the rangefinder): the floor's true depth in image row v is h * cos(a) / sin(a + pitch), a = atan((v - cy) / f).
    The median of (true / DA3) over the floor rows that agree is the scale for this frame; smoothed; kept when the floor is not usable (tilted, too low, cluttered)."""

    def __init__(self, alpha=0.4, lo=0.7, hi=1.3):
        self.s = 1.0; self.alpha = alpha; self.lo, self.hi = lo, hi; self.used = 0; self.last_ok = 0.0

    def update(self, depth, h_cam, pitch_rad=0.0, roll_rad=0.0, f_px=None, cy=None):
        """depth: metres (H x W), h_cam: camera height above the floor (m), pitch + = nose down. Returns the scale to MULTIPLY the depth by."""
        H, W = depth.shape
        if h_cam is None or h_cam < 0.25 or abs(pitch_rad) > 0.12 or abs(roll_rad) > 0.06: return self.s
        f = f_px if f_px else 292.2957 * H / 480.0; cy = cy if cy is not None else H / 2.0; rat = []      # VERTICAL focal length in depth-map pixels (DA3 outputs 504x336: not the 4:3 aspect of the picture)
        for v in range(int(cy + 0.2 * H), H - 2, max(1, H // 120)):
            a = np.arctan((v - cy) / f); el = a + pitch_rad
            if el < 0.15: continue
            exp = h_cam * np.cos(a) / np.sin(el); got = np.median(depth[v, int(0.3 * W):int(0.7 * W)])
            if 0.05 < got < 6.0 and exp < 3.5: rat.append(exp / got)
        if len(rat) < 8: return self.s
        r = np.array(rat); med = float(np.median(r)); good = r[np.abs(r - med) < 0.15 * med]
        if len(good) < 8: return self.s
        new = float(np.clip(np.median(good), self.lo, self.hi)); self.s += self.alpha * (new - self.s); self.used += 1; self.last_ok = time.time()
        return self.s


def grid_stats(depth):
    h, w = depth.shape; mn = np.full((ROWS, COLS), np.nan); me = mn.copy(); mx = mn.copy(); p10 = mn.copy(); cnt = np.zeros((ROWS, COLS), int)
    for r in range(ROWS):
        for c in range(COLS):
            cell = depth[r * h // ROWS:(r + 1) * h // ROWS, c * w // COLS:(c + 1) * w // COLS]; v = cell[np.isfinite(cell) & (cell > 0)]
            if v.size: mn[r, c], me[r, c], mx[r, c], p10[r, c], cnt[r, c] = v.min(), v.mean(), v.max(), np.percentile(v, 10), v.size
    return mn, me, mx, p10, cnt


# Nimbus Air75 depth profile (reference 640x480), scaled per axis to the depth map size exactly as the Nimbus core does (analysis doc 06, section 1.2)
FX, FY, CX, CY = 292.2957, 305.1782, 316.2907, 259.7271


def voxel_floorplan(depth, vox=0.125, stride=2, zmax=6.0, xhalf=3.0, band=(-1.5, 0.0), min_pts=3):
    """Nimbus-style live voxel floor plan: back-project the depth map with the Air75 intrinsics, keep the points inside a height band, flatten top-down
    into vox-sized cells. Per frame, camera frame, no pose (like Nimbus' VoxelizedFloorplanLoop; voxel 0.125 m and stride 2 are its values).
    band = height of a point relative to the camera in metres, +down: from 1.5 m above the camera down to camera level, so the floor (which DA3 reads too near) and anything the drone
    would fly over is left out (my choice: the Nimbus band floats 0.35 / 10.0 are not understood yet).
    Returns a sparse description: {"vox", "nx", "nz", "cells": [[ix, iz, count], ...]} with ix across (0 = left), iz forward."""
    H, W = depth.shape; fx, fy, cx, cy = FX * W / 640.0, FY * H / 480.0, CX * W / 640.0, CY * H / 480.0
    v, u = np.mgrid[0:H:stride, 0:W:stride]; d = depth[::stride, ::stride]
    ok = np.isfinite(d) & (d > 0.1) & (d < zmax)
    X = (u - cx) * d / fx; Y = (v - cy) * d / fy
    ok &= (Y >= band[0]) & (Y <= band[1]) & (np.abs(X) < xhalf)
    nx, nz = int(round(2 * xhalf / vox)), int(round(zmax / vox))
    ix = np.floor((X[ok] + xhalf) / vox).astype(int); iz = np.floor(d[ok] / vox).astype(int)
    cnt = np.bincount(iz * nx + ix, minlength=nx * nz)
    idx = np.nonzero(cnt >= min_pts)[0]
    return {"vox": vox, "nx": nx, "nz": nz, "cells": [[int(i % nx), int(i // nx), int(min(cnt[i], 255))] for i in idx]}


def render_floorplan_cv(fp, size=(480, 480)):
    """Draw the sparse floor plan (BGR image): drone at the bottom centre, forward is up, rings every 1 m, cells nearer than 1 m outlined as obstacles."""
    W, H = size; img = np.full((H, W, 3), 28, np.uint8); nx, nz, vox = fp["nx"], fp["nz"], fp["vox"]
    sx, sz = W / nx, H / nz
    for m in range(1, int(nz * vox) + 1):
        cv2.line(img, (0, H - int(m / vox * sz)), (W, H - int(m / vox * sz)), (55, 55, 55), 1); cv2.putText(img, f"{m} m", (4, H - int(m / vox * sz) - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (110, 110, 110), 1, cv2.LINE_AA)
    cv2.line(img, (W // 2, 0), (W // 2, H), (55, 55, 55), 1)
    for ix, iz, c in fp["cells"]:
        x0, y0 = int(ix * sx), H - int((iz + 1) * sz); col = (255, int(150 + min(c, 60)), 90) if iz * vox > 1.0 else (40, 170, 240)   # soft blue; amber (BGR) inside 1 m
        cv2.rectangle(img, (x0, y0), (x0 + max(1, int(sx)), y0 + max(1, int(sz))), col, -1)
    cv2.drawMarker(img, (W // 2, H - 6), (230, 230, 230), cv2.MARKER_TRIANGLE_UP, 14, 2)
    return img


def result_json(d, dt, faces=None):
    mn, me, mx, p10, cnt = grid_stats(d)
    valid = np.isfinite(d) & (d >= 0.2) & (d <= 1.0); nsamp = int(valid.sum()); nearest = float(d[valid].min()) if nsamp else None
    f = lambda A: [9.0 if not np.isfinite(v) else round(float(v), 2) for v in A.ravel()]
    return {"rows": ROWS, "cols": COLS, "min_m": f(mn), "mean_m": f(me), "max_m": f(mx), "p10_m": f(p10), "count": cnt.ravel().tolist(),
            "nearest_m": None if nearest is None else round(nearest, 2), "obstacle": nsamp > 19, "obstacle_samples": nsamp, "faces": faces, "latency_ms": round(dt * 1000), "floorplan": voxel_floorplan(d)}, mn, p10, nearest
