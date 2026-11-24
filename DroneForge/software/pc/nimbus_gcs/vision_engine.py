"""GCS vision engines (Qt-free): the models run on the PC on the live video (O4 / the simulator's O4 feed), as Nimbus does on its ground station.

    DEPTH     Depth Anything 3 metric (models/nimbus/da3_metric.onnx): colour depth overlay + nearest distance
    DETECT    obstacles from the depth (Nimbus obstacle_detected: cells nearer than 1.0 m, 6x8 grid); with models/detect.onnx (a YOLOv8/YOLO11 COCO ONNX) people/objects instead
    FACE REC  InsightFace buffalo_l: SCRFD face detection (det_10g.onnx) + ArcFace recognition (w600k_r50.onnx) against the saved faces (~/.config/droneforge/faces)
    TRACK     follows one box (a recognised face, the largest face, or a box dragged on the video) with OpenCV's MIL tracker
    VO        SuperPoint + LightGlue RGB-D visual odometry on DA3 depth: the drone's displacement from the hover key frame -> the v4.7 FC position aid (MSP2_DF_POS_AID)

One worker thread takes the NEWEST frame each round and runs what is switched on; annotate() draws the latest results on any frame (cheap, called by the video views).
Everything loads lazily on the first switch-on; a missing model or GPU library becomes a plain status text, never an exception in the GUI. Camera model: the VO assumes the
640x480 Nimbus-profile picture (fx 292.3 px), which the simulator's O4 feed is; the real O4 needs the lens calibration (tools_scripts/o4_calibrate.py) first (UNVERIFIED)."""
from __future__ import annotations

import json
import math
import os
import sys
import threading
import time
from typing import Callable, Optional

import numpy as np

try:
    import cv2
except Exception:                                                     # noqa: BLE001
    cv2 = None

DF = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
MODELS = os.path.join(DF, "gazebo", "vision", "models")
INSIGHT = os.path.expanduser("~/.insightface/models/buffalo_l")
FACES_DIR = os.path.expanduser("~/.config/droneforge/faces")
ENGINES = ("depth", "detect", "face", "track", "vo")
LABELS = {"depth": "DEPTH", "detect": "DETECT", "face": "FACE REC", "track": "TRACK", "vo": "VO"}
CAM_LEVER_M = 0.065                                                   # O4 camera ahead of the body centre (sim model; the real mount: same design)
ARC_DST = np.array([[38.2946, 51.6963], [73.5318, 51.5014], [56.0252, 71.7366], [41.5493, 92.3655], [70.7299, 92.2041]], np.float32)   # ArcFace 112x112 landmarks


def _ort():
    import onnxruntime as ort
    try:
        ort.preload_dlls()
    except Exception:                                                 # noqa: BLE001
        pass
    prov = [p for p in ("CUDAExecutionProvider", "CPUExecutionProvider") if p in ort.get_available_providers()]
    return ort, prov


class _Scrfd:
    """InsightFace SCRFD (det_10g): faces + 5 landmarks."""
    def __init__(self, path):
        ort, prov = _ort(); self.s = ort.InferenceSession(path, providers=prov); self.inp = self.s.get_inputs()[0].name; self.size = 640

    def __call__(self, bgr, thr=0.5):
        h, w = bgr.shape[:2]; sc = self.size / max(h, w); nw, nh = int(w * sc), int(h * sc)
        can = np.zeros((self.size, self.size, 3), np.uint8); can[:nh, :nw] = cv2.resize(bgr, (nw, nh))
        x = ((can[..., ::-1].astype(np.float32) - 127.5) / 128.0).transpose(2, 0, 1)[None]
        out = self.s.run(None, {self.inp: x}); faces = []
        for i, st in enumerate((8, 16, 32)):
            sc_, bb, kp = out[i].reshape(-1), out[i + 3].reshape(-1, 4) * st, out[i + 6].reshape(-1, 10) * st
            n = self.size // st; cy, cx = np.mgrid[0:n, 0:n]; ctr = np.repeat(np.stack([cx.ravel(), cy.ravel()], 1) * st, 2, axis=0).astype(np.float32)
            for j in np.nonzero(sc_ > thr)[0]:
                c = ctr[j]; box = np.array([c[0] - bb[j, 0], c[1] - bb[j, 1], c[0] + bb[j, 2], c[1] + bb[j, 3]]) / sc
                faces.append((float(sc_[j]), box, (c + kp[j].reshape(5, 2)) / sc))
        faces.sort(key=lambda f: -f[0]); keep = []
        for f in faces:                                              # NMS
            if all(_iou(f[1], k[1]) < 0.4 for k in keep): keep.append(f)
        return keep


class _ArcFace:
    def __init__(self, path):
        ort, prov = _ort(); self.s = ort.InferenceSession(path, providers=prov); self.inp = self.s.get_inputs()[0].name

    def __call__(self, bgr, kps):
        M, _ = cv2.estimateAffinePartial2D(np.asarray(kps, np.float32), ARC_DST)
        if M is None: return None
        crop = cv2.warpAffine(bgr, M, (112, 112))
        x = ((crop[..., ::-1].astype(np.float32) - 127.5) / 127.5).transpose(2, 0, 1)[None]
        e = self.s.run(None, {self.inp: x})[0][0]; return e / (np.linalg.norm(e) + 1e-9)


class _Yolo:
    """YOLOv8/YOLO11 COCO ONNX ([1, 84, N] output), only if models/detect.onnx exists."""
    COCO = ("person bicycle car motorcycle airplane bus train truck boat traffic_light fire_hydrant stop_sign parking_meter bench bird cat dog horse sheep cow elephant bear zebra "
            "giraffe backpack umbrella handbag tie suitcase frisbee skis snowboard sports_ball kite baseball_bat baseball_glove skateboard surfboard tennis_racket bottle wine_glass cup "
            "fork knife spoon bowl banana apple sandwich orange broccoli carrot hot_dog pizza donut cake chair couch potted_plant bed dining_table toilet tv laptop mouse remote keyboard "
            "cell_phone microwave oven toaster sink refrigerator book clock vase scissors teddy_bear hair_drier toothbrush").split()

    def __init__(self, path):
        ort, prov = _ort(); self.s = ort.InferenceSession(path, providers=prov); self.inp = self.s.get_inputs()[0].name

    def __call__(self, bgr, thr=0.4):
        h, w = bgr.shape[:2]; sc = 640 / max(h, w); nw, nh = int(w * sc), int(h * sc)
        can = np.full((640, 640, 3), 114, np.uint8); can[:nh, :nw] = cv2.resize(bgr, (nw, nh))
        o = self.s.run(None, {self.inp: (can[..., ::-1].astype(np.float32) / 255.0).transpose(2, 0, 1)[None]})[0][0].T
        cls = o[:, 4:].argmax(1); conf = o[np.arange(len(o)), 4 + cls]; dets = []
        for j in np.argsort(-conf)[:300]:
            if conf[j] < thr: break
            cx, cy, bw, bh = o[j, :4] / sc; box = np.array([cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2])
            if all(_iou(box, d[1]) < 0.45 or d[2] != cls[j] for d in dets): dets.append((float(conf[j]), box, int(cls[j])))
        return [(c, b, self.COCO[k] if k < len(self.COCO) else str(k)) for c, b, k in dets]


def _iou(a, b):
    x1, y1, x2, y2 = max(a[0], b[0]), max(a[1], b[1]), min(a[2], b[2]), min(a[3], b[3])
    i = max(0.0, x2 - x1) * max(0.0, y2 - y1); u = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - i
    return i / u if u > 0 else 0.0


def vo_to_body(d_fwd: float, d_left: float, yaw_left: float, lever: float = CAM_LEVER_M):
    """Camera displacement since the key frame (key-frame axes: forward, left; yaw change counter-clockwise) -> the BODY CENTRE's displacement in the drone's CURRENT axes
    (forward, right), as the FC position aid wants it."""
    c, s = math.cos(yaw_left), math.sin(yaw_left)
    bf, bl = d_fwd - lever * (c - 1.0), d_left - lever * s           # remove the camera lever arm swung by the yaw
    f = c * bf + s * bl; l = -s * bf + c * bl                         # into the current axes
    return f, -l


class VisionEngine:
    def __init__(self, get_frame: Callable, models_dir: str = MODELS, faces_dir: str = FACES_DIR) -> None:
        self.get_frame = get_frame; self.models = models_dir; self.faces_dir = faces_dir
        self.on = {k: False for k in ENGINES}; self.state = {k: "off" for k in ENGINES}; self.text = {k: "" for k in ENGINES}; self.hz = {k: 0.0 for k in ENGINES}
        self._lock = threading.Lock(); self._stop = threading.Event(); self._thread: Optional[threading.Thread] = None; self._load_lock = threading.Lock(); self._threads = []
        self._da3 = self._vo = self._scrfd = self._arc = self._yolo = None; self._tracker = None; self._track_req = None
        self.depth = None; self.depth_t = 0.0; self.obstacles = []; self.detections = []; self.faces = []; self.track_box = None; self.vo_pose = None
        self._anchor_req = False; self.anchor_t = 0.0; self.aid = None; self.aid_t = 0.0; self.vo_inliers = 0
        self.floor = None; self.pose = None                                           # floor-based DA3 scale (nimbus_depth_core.FloorScale) from the FC's height + attitude
        self.known = self._load_faces()

    # ---- switches
    def set(self, name: str, on: bool) -> str:
        if name not in ENGINES: return "unknown engine"
        with self._lock:
            self.on[name] = bool(on)
            if not on:
                self.state[name] = "off"; self.text[name] = ""; self.hz[name] = 0.0
                if name == "track": self._tracker = None; self.track_box = None
                if name == "vo": self.clear_anchor()
            else:
                self.state[name] = "loading"; self.text[name] = "loading the model..."
        if on and self._thread is None:
            self._stop.clear(); self._threads = []
            for grp in ("depth", "face", "track", "vo"):                     # one worker per engine: the GPU models and the CPU tracker run in parallel, each on the newest frame
                th = threading.Thread(target=self._run, args=(grp,), name=f"gcs-vision-{grp}", daemon=True); th.start(); self._threads.append(th)
            self._thread = self._threads[0]
        return f"{LABELS[name]} {'on' if on else 'off'}"

    def any_on(self) -> bool:
        return any(self.on.values())

    def stop(self) -> None:
        self._stop.set()
        for th in getattr(self, "_threads", []): th.join(timeout=2.0)
        self._thread = None; self._threads = []

    # ---- TRACK / FACE / VO requests from the GUI
    def track_box_request(self, box) -> None:
        """Start tracking this box (x1, y1, x2, y2 in frame pixels): a box dragged on the video, or a face."""
        self._track_req = tuple(float(v) for v in box)

    def enroll(self, name: str) -> str:
        name = "".join(ch for ch in name.strip() if ch.isalnum() or ch in "-_ ").strip()[:32]
        if not name: return "Type a name first."
        with self._lock:
            f = max(self.faces, key=lambda x: (x["box"][2] - x["box"][0]) * (x["box"][3] - x["box"][1]), default=None)
        if f is None or f.get("emb") is None: return "No face in view to save."
        os.makedirs(self.faces_dir, exist_ok=True); np.save(os.path.join(self.faces_dir, name + ".npy"), f["emb"]); self.known = self._load_faces()
        return f"Saved the face as {name} ({len(self.known)} known)."

    def request_anchor(self) -> None:
        self._anchor_req = True

    def clear_anchor(self) -> None:
        self._anchor_req = False; self.aid = None; self.anchor_t = 0.0
        if self._vo is not None: self._vo.anchor = None

    def set_pose(self, h_cam, pitch_rad, roll_rad) -> None:
        """The camera height above the floor (FC range) and the attitude (pitch + = nose down): DA3's metric scale is corrected on the floor with them."""
        self.pose = (h_cam, pitch_rad, roll_rad)

    def has_anchor(self) -> bool:
        return self._vo is not None and self._vo.anchor is not None

    def anchor_pending(self) -> bool:
        return self._anchor_req

    def fresh_aid(self, max_age: float = 0.25):
        a, t = self.aid, self.aid_t
        return a if (a is not None and time.time() - t < max_age) else None

    # ---- worker
    def _load_faces(self) -> dict:
        out = {}
        try:
            for f in os.listdir(self.faces_dir):
                if f.endswith(".npy"): out[f[:-4]] = np.load(os.path.join(self.faces_dir, f))
        except OSError:
            pass
        return out

    def _ensure(self, name: str) -> bool:
        with self._load_lock:                                          # the workers start in parallel: load each model once
            return self._ensure_locked(name)

    def _ensure_locked(self, name: str) -> bool:
        try:
            if name in ("depth", "detect", "vo") and self._da3 is None:
                sys.path.insert(0, os.path.join(DF, "gazebo", "vision")); from nimbus_depth_core import Da3          # noqa: E402
                self._da3 = Da3(4)
            if name == "detect" and self._yolo is None and os.path.exists(os.path.join(self.models, "detect.onnx")):
                self._yolo = _Yolo(os.path.join(self.models, "detect.onnx"))
            if name in ("face", "track") and self._scrfd is None:
                if not os.path.exists(os.path.join(INSIGHT, "det_10g.onnx")): raise FileNotFoundError(f"face model missing: {INSIGHT}/det_10g.onnx")
                self._scrfd = _Scrfd(os.path.join(INSIGHT, "det_10g.onnx"))
                if os.path.exists(os.path.join(INSIGHT, "w600k_r50.onnx")): self._arc = _ArcFace(os.path.join(INSIGHT, "w600k_r50.onnx"))
            if name == "vo" and self._vo is None:
                sys.path.insert(0, os.path.join(DF, "gazebo", "vision")); from visual_odom import RgbdVo                 # noqa: E402
                self._vo = RgbdVo(self.models)
            return True
        except Exception as e:                                        # noqa: BLE001
            with self._lock: self.state[name] = "error"; self.text[name] = f"cannot start: {e}"[:160]
            return False

    GROUPS = {"depth": ("depth", "detect"), "face": ("face",), "track": ("track",), "vo": ("vo",)}

    def _run(self, grp: str) -> None:
        last_t = 0.0; tick = {k: [] for k in ENGINES}; mine = self.GROUPS[grp]
        while not self._stop.is_set():
            on = dict(self.on)
            active = any(on[k] for k in mine) or (grp == "depth" and on["vo"] and (self._anchor_req or self._da3 is None))   # VO needs depth only for its key frame
            if not active: time.sleep(0.05); continue
            for k in mine + (("vo",) if grp == "depth" and on["vo"] else ()):
                if on[k] and self.state[k] == "loading": self._ensure(k)
            try:
                fr = self.get_frame()
            except Exception:                                         # noqa: BLE001
                fr = (None, 0.0)
            frame, t = fr if isinstance(fr, tuple) else (fr, time.time())
            if frame is None or t == last_t: time.sleep(0.003); continue
            last_t = t
            try:
                if grp == "depth": self._step_depth(frame, on, tick)
                elif grp == "face": self._step_face(frame, on, tick)
                elif grp == "track": self._track(frame, tick)
                elif grp == "vo" and self._vo is not None: self._vo_step(frame, tick)
            except Exception as e:                                    # noqa: BLE001
                with self._lock:
                    for k in mine:
                        if on[k] and self.state[k] != "error": self.text[k] = f"error: {e}"[:160]
                time.sleep(0.2)

    def _mark(self, k, tick, text):
        now = time.time(); tick[k] = [x for x in tick[k] if now - x < 2.0] + [now]
        with self._lock:
            if self.on[k] and self.state[k] != "error": self.state[k] = "on"; self.text[k] = text; self.hz[k] = len(tick[k]) / 2.0

    def _step_depth(self, bgr, on, tick) -> None:
        if self._da3 is None: return
        depth = self._da3(bgr[..., ::-1].copy())
        if self.pose is not None:                                                     # per-frame metric scale from the floor (DA3 jumps +-13 % frame to frame)
            if self.floor is None:
                from nimbus_depth_core import FloorScale
                self.floor = FloorScale()
            depth = depth * self.floor.update(depth, *self.pose)
        self.depth = depth; self.depth_t = time.time()
        if on["depth"]:
            dv = depth[np.isfinite(depth) & (depth > 0.1)]
            self._mark("depth", tick, f"nearest {np.percentile(dv, 1):.2f} m" if dv.size else "no depth")
        if on["detect"]:
            if self._yolo is not None:
                dets = self._yolo(bgr); self.detections = dets
                self._mark("detect", tick, f"{len(dets)} object(s)" + (f": {dets[0][2]}" if dets else ""))
            else:
                obs = []; h, w = depth.shape; fy = 305.1782 * h / 480.0; cy = 259.7271 * h / 480.0      # Nimbus-profile intrinsics scaled to the depth map
                for r in range(6):
                    for c in range(8):
                        cell = depth[r * h // 6:(r + 1) * h // 6, c * w // 8:(c + 1) * w // 8]; v = cell[np.isfinite(cell) & (cell > 0.15)]
                        if not v.size: continue
                        d = float(np.percentile(v, 10)); vc = (r + 0.5) * h / 6.0
                        if d < 1.0 and (vc - cy) * d / fy < 0.3: obs.append((r, c, d))      # not the floor: a point more than 0.3 m below the (level) camera is ground
                self.obstacles = obs
                self._mark("detect", tick, (f"OBSTACLE {min(o[2] for o in obs):.2f} m" if obs else "clear (nothing within 1.0 m)") + " (depth; add models/detect.onnx for objects)")

    def _step_face(self, bgr, on, tick) -> None:
        if self._scrfd is None: return
        faces = []
        for sc, box, kps in self._scrfd(bgr):
            emb = self._arc(bgr, kps) if self._arc is not None else None; who, sim = None, 0.0
            if emb is not None and self.known:
                who, sim = max(((n, float(np.dot(emb, e))) for n, e in self.known.items()), key=lambda x: x[1])
                if sim < 0.35: who = None
            faces.append(dict(box=box, score=sc, emb=emb, who=who, sim=sim))
        self.faces = faces
        names = [f["who"] for f in faces if f["who"]]
        self._mark("face", tick, f"{len(faces)} face(s)" + (": " + ", ".join(names) if names else ("" if self.known else " (no saved faces yet)")))

    def _track(self, bgr, tick) -> None:
        req, self._track_req = self._track_req, None
        if req is None and self._tracker is None:                    # nothing chosen: lock onto a recognised face, else the largest face
            cand = sorted(self.faces, key=lambda f: (f["who"] is None, -(f["box"][2] - f["box"][0])))
            if cand: req = tuple(cand[0]["box"])
        small = cv2.resize(bgr, (bgr.shape[1] // 2, bgr.shape[0] // 2), interpolation=cv2.INTER_AREA)     # half resolution: the MIL tracker is CPU-bound
        if req is not None:
            x1, y1, x2, y2 = [v / 2.0 for v in req]; tr = cv2.TrackerMIL_create(); tr.init(small, (int(x1), int(y1), max(4, int(x2 - x1)), max(4, int(y2 - y1)))); self._tracker = tr
        if self._tracker is None:
            self._mark("track", tick, "drag a box on the video, or turn FACE REC on"); return
        ok, b = self._tracker.update(small)
        if ok:
            b = [v * 2.0 for v in b]
            self.track_box = (b[0], b[1], b[0] + b[2], b[1] + b[3]); cx = (b[0] + b[2] / 2) / bgr.shape[1] - 0.5
            self._mark("track", tick, f"tracking: {'left' if cx < -0.05 else 'right' if cx > 0.05 else 'centre'} {abs(cx) * 100:.0f} %")
        else:
            self.track_box = None; self._tracker = None; self._mark("track", tick, "lost the target")

    def _vo_step(self, bgr, tick) -> None:
        if bgr.shape[:2] != (480, 640):
            self._mark("vo", tick, f"needs the 640x480 rectified picture (got {bgr.shape[1]}x{bgr.shape[0]})"); return
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        if self._anchor_req:                                          # the key frame needs depth: the newest DA3 map (the drone hovers still, so 0.2 s old is fine)
            depth = self.depth
            if depth is None or time.time() - self.depth_t > 0.3: self._mark("vo", tick, "waiting for a depth map for the key frame"); return
            dmm = (cv2.resize(depth, (640, 480), interpolation=cv2.INTER_NEAREST) * 1000.0).astype(np.uint16)
            n = self._vo.set_anchor(gray, dmm, time.time()); self._anchor_req = False; self.anchor_t = time.time(); self.aid = (0.0, 0.0); self.aid_t = time.time()
            self._mark("vo", tick, f"key frame set ({n} points with depth)"); return
        if self._vo.anchor is None:
            self._mark("vo", tick, "ready: the key frame is set when the drone hovers still"); return
        r, n = self._vo.track(gray)
        self.vo_inliers = n
        if r is None or r[4] < 25:
            self.aid = None; self._mark("vo", tick, f"weak match ({n} matches): no position aid"); return
        d_fwd, d_left, d_up, yaw_left, inl = r
        if abs(d_up) > 0.5 or math.hypot(d_fwd, d_left) > 3.0:
            self.aid = None; self._mark("vo", tick, "implausible jump: no position aid"); return
        f, rt = vo_to_body(d_fwd, d_left, yaw_left); self.aid = (f, rt); self.aid_t = time.time(); self.vo_pose = (d_fwd, d_left, yaw_left)
        self._mark("vo", tick, f"from key frame: {f:+.2f} m fwd, {rt:+.2f} m right ({inl} inliers)")

    # ---- drawing (GUI thread, any frame)
    def annotate(self, frame):
        if frame is None or cv2 is None or not self.any_on(): return frame
        out = frame.copy(); h, w = out.shape[:2]
        if self.on["depth"] and self.depth is not None and time.time() - self.depth_t < 1.0:
            d = cv2.resize(self.depth, (w, h)); v = np.clip(1.0 - (d - 0.2) / 4.8, 0, 1)
            col = cv2.applyColorMap((v * 255).astype(np.uint8), cv2.COLORMAP_TURBO); out = cv2.addWeighted(out, 0.55, col, 0.45, 0)
        if self.on["detect"]:
            if self._yolo is not None:
                for c, b, lab in self.detections: _box(out, b, f"{lab} {c:.2f}", (255, 214, 185))
            else:
                for r, c, dist in self.obstacles:
                    x1, y1, x2, y2 = c * w // 8, r * h // 6, (c + 1) * w // 8, (r + 1) * h // 6
                    cv2.rectangle(out, (x1 + 2, y1 + 2), (x2 - 2, y2 - 2), (77, 185, 240) if dist > 0.6 else (95, 90, 239), 2)
                    cv2.putText(out, f"{dist:.1f}", (x1 + 6, y1 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (240, 240, 240), 1, cv2.LINE_AA)
        if self.on["face"]:
            for f in self.faces: _box(out, f["box"], f"{f['who']} {f['sim']:.2f}" if f["who"] else "face", (255, 214, 185))
        if self.on["track"] and self.track_box is not None:
            _box(out, self.track_box, "TRACK", (255, 214, 185), 3)
        if self.on["vo"] and self.aid is not None:
            cv2.putText(out, f"VO {self.aid[0]:+.2f} m fwd {self.aid[1]:+.2f} m right", (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 214, 185), 1, cv2.LINE_AA)
        return out


def _box(img, b, label, col, thick=2):
    x1, y1, x2, y2 = [int(v) for v in b]
    cv2.rectangle(img, (x1, y1), (x2, y2), col, thick)
    cv2.putText(img, label, (x1, max(14, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv2.LINE_AA)


# ------------------------------------------------------------------------------------------------------------------------------------------------------------------
# The engine in its OWN process: the GCS process keeps the video and the GUI at 30 fps (in one process the models and the decoder shared Python's lock: 17 fps).
# The GCS copies each new frame into shared memory; the child runs VisionEngine on it and sends back a small snapshot (~30 Hz) that annotate() draws.

MAX_W, MAX_H = 1920, 1088


def _child(shm_name, meta, cmd_q, out_q, stop_ev):
    from multiprocessing import shared_memory
    shm = shared_memory.SharedMemory(name=shm_name); buf = np.ndarray((MAX_H, MAX_W, 3), np.uint8, buffer=shm.buf)
    def get_frame():
        with meta.get_lock():
            seq, h, w, t = int(meta[0]), int(meta[1]), int(meta[2]), float(meta[3])
            if seq == 0 or h == 0: return (None, 0.0)
            f = buf[:h, :w].copy()
        return (f, t)
    e = VisionEngine(get_frame); last = 0.0
    while not stop_ev.is_set():
        try:
            while True:
                c = cmd_q.get_nowait(); op = c[0]
                if op == "set": e.set(c[1], c[2])
                elif op == "box": e.track_box_request(c[1])
                elif op == "enroll": out_q.put(("msg", e.enroll(c[1])))
                elif op == "anchor": e.request_anchor()
                elif op == "clear": e.clear_anchor()
                elif op == "pose": e.set_pose(*c[1])
        except Exception:                                             # noqa: BLE001  (queue.Empty)
            pass
        now = time.time()
        if now - last >= 0.033:
            last = now
            dsm = None
            if e.on["depth"] and e.depth is not None and now - e.depth_t < 1.0 and cv2 is not None:
                dsm = cv2.resize(e.depth, (160, 120)).astype(np.float16)
            snap = dict(on=dict(e.on), state=dict(e.state), text=dict(e.text), hz=dict(e.hz), depth=dsm, depth_t=e.depth_t, obstacles=list(e.obstacles),
                        detections=[(c_, [float(v) for v in b], l) for c_, b, l in e.detections], yolo=e._yolo is not None,
                        faces=[dict(box=[float(v) for v in f["box"]], who=f["who"], sim=f["sim"]) for f in e.faces], track_box=e.track_box,
                        aid=e.aid, aid_t=e.aid_t, has_anchor=e.has_anchor(), anchor_req=e.anchor_pending(), known=len(e.known))
            try:
                while out_q.qsize() > 2: out_q.get_nowait()
            except Exception:                                         # noqa: BLE001
                pass
            out_q.put(("snap", snap))
        time.sleep(0.005)
    e.stop(); shm.close()


class VisionProcess:
    """Same interface as VisionEngine for the GCS (vision_bar.py, the video views); the models run in a child process."""

    def __init__(self, get_frame: Callable) -> None:
        self.get_frame = get_frame; self.on = {k: False for k in ENGINES}; self.state = {k: "off" for k in ENGINES}; self.text = {k: "" for k in ENGINES}; self.hz = {k: 0.0 for k in ENGINES}
        self.snap = {}; self.msg = ""; self._proc = None; self._stop = threading.Event(); self._pump = None
        self._last_t = 0.0

    def _start(self) -> None:
        import multiprocessing as mp
        from multiprocessing import shared_memory
        ctx = mp.get_context("spawn")
        self._shm = shared_memory.SharedMemory(create=True, size=MAX_W * MAX_H * 3); self._buf = np.ndarray((MAX_H, MAX_W, 3), np.uint8, buffer=self._shm.buf)
        self._meta = ctx.Array("d", 4); self._cmd = ctx.Queue(); self._out = ctx.Queue(); self._stop_ev = ctx.Event()
        self._proc = ctx.Process(target=_child, args=(self._shm.name, self._meta, self._cmd, self._out, self._stop_ev), name="gcs-vision", daemon=True); self._proc.start()
        self._stop.clear(); self._pump = threading.Thread(target=self._pump_loop, name="gcs-vision-pump", daemon=True); self._pump.start()

    def _pump_loop(self) -> None:
        seq = 0
        while not self._stop.is_set():
            try:
                fr = self.get_frame(); f, t = fr if isinstance(fr, tuple) else (fr, time.time())
            except Exception:                                         # noqa: BLE001
                f, t = None, 0.0
            if f is not None and t != self._last_t and f.ndim == 3 and f.shape[0] <= MAX_H and f.shape[1] <= MAX_W:
                self._last_t = t; h, w = f.shape[:2]; seq += 1
                with self._meta.get_lock():
                    self._buf[:h, :w] = f; self._meta[0] = seq; self._meta[1] = h; self._meta[2] = w; self._meta[3] = t
            try:
                while True:
                    kind, v = self._out.get_nowait()
                    if kind == "snap":
                        self.snap = v; self.state = v["state"]; self.text = v["text"]; self.hz = v["hz"]
                    else:
                        self.msg = v
            except Exception:                                         # noqa: BLE001  (queue.Empty)
                pass
            time.sleep(0.004)

    def set(self, name: str, on: bool) -> str:
        if name not in ENGINES: return "unknown engine"
        if on and self._proc is None: self._start()
        self.on[name] = bool(on); self.state[name] = "loading" if on else "off"; self.text[name] = "loading the model..." if on else ""
        if self._proc is not None: self._cmd.put(("set", name, bool(on)))
        return f"{LABELS[name]} {'on' if on else 'off'}"

    def any_on(self) -> bool:
        return any(self.on.values())

    def track_box_request(self, box) -> None:
        if self._proc is not None: self._cmd.put(("box", tuple(float(v) for v in box)))

    def enroll(self, name: str) -> str:
        if self._proc is None: return "Turn FACE REC on first."
        self._cmd.put(("enroll", name)); return "Saving the face..."

    def request_anchor(self) -> None:
        if self._proc is not None: self._cmd.put(("anchor",)); self.snap["anchor_req"] = True

    def clear_anchor(self) -> None:
        if self._proc is not None: self._cmd.put(("clear",)); self.snap["has_anchor"] = False; self.snap["aid"] = None

    def set_pose(self, h_cam, pitch_rad, roll_rad) -> None:
        if self._proc is not None: self._cmd.put(("pose", (h_cam, pitch_rad, roll_rad)))

    def has_anchor(self) -> bool:
        return bool(self.snap.get("has_anchor"))

    def anchor_pending(self) -> bool:
        return bool(self.snap.get("anchor_req"))

    def fresh_aid(self, max_age: float = 0.25):
        a, t = self.snap.get("aid"), self.snap.get("aid_t", 0.0)
        return a if (a is not None and time.time() - t < max_age) else None

    def stop(self) -> None:
        self._stop.set()
        if self._proc is not None:
            self._stop_ev.set(); self._proc.join(timeout=3.0)
            if self._proc.is_alive(): self._proc.terminate()
            self._proc = None
            try: self._shm.close(); self._shm.unlink()
            except Exception: pass                                    # noqa: BLE001

    def annotate(self, frame):
        s = self.snap
        if frame is None or cv2 is None or not self.any_on() or not s: return frame
        out = frame.copy(); h, w = out.shape[:2]; on = self.on
        if on["depth"] and s.get("depth") is not None and time.time() - s.get("depth_t", 0) < 1.0:
            d = cv2.resize(s["depth"].astype(np.float32), (w, h)); v = np.clip(1.0 - (d - 0.2) / 4.8, 0, 1)
            out = cv2.addWeighted(out, 0.55, cv2.applyColorMap((v * 255).astype(np.uint8), cv2.COLORMAP_TURBO), 0.45, 0)
        if on["detect"]:
            if s.get("yolo"):
                for c, b, lab in s.get("detections", []): _box(out, b, f"{lab} {c:.2f}", (255, 214, 185))
            else:
                for r, c, dist in s.get("obstacles", []):
                    x1, y1, x2, y2 = c * w // 8, r * h // 6, (c + 1) * w // 8, (r + 1) * h // 6
                    cv2.rectangle(out, (x1 + 2, y1 + 2), (x2 - 2, y2 - 2), (77, 185, 240) if dist > 0.6 else (95, 90, 239), 2)
                    cv2.putText(out, f"{dist:.1f} m", (x1 + 6, y1 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (240, 240, 240), 1, cv2.LINE_AA)
        if on["face"]:
            for f in s.get("faces", []): _box(out, f["box"], f"{f['who']} {f['sim']:.2f}" if f["who"] else "face", (255, 214, 185))
        if on["track"] and s.get("track_box") is not None: _box(out, s["track_box"], "TRACK", (255, 214, 185), 3)
        if on["vo"] and self.fresh_aid(0.5) is not None:
            a = self.fresh_aid(0.5); cv2.putText(out, f"VO {a[0]:+.2f} m fwd {a[1]:+.2f} m right", (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 214, 185), 1, cv2.LINE_AA)
        return out
