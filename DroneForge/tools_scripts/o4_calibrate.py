#!/usr/bin/env python3
"""O4 camera tool for the real drone's video feed (CosmoStreamer RTSP): latency, lens calibration, and a live check of the rectifier. READ-ONLY: it only reads the video
stream; it never touches the flight controller, the radio or the drone's motors, and it arms nothing.

  latency    measure the camera-to-PC delay in ms (mean, jitter) with the QR-code method the GCS Video page uses: this tool shows a QR code carrying the PC's clock, you point the
             O4 at that screen, and the delay of every decoded frame is recorded. Writes gazebo/config/o4_latency.json (run_house.sh uses it as the simulated feed latency).
  calibrate  checkerboard lens calibration: hold a printed checkerboard in front of the O4, the tool picks well-spread views by itself, fits the lens model (equidistant fisheye for
             the O4 Pro / Wide, pinhole for the standard O4), reports the reprojection error and the REAL field of view of your stream, and (with --write) stores the result in
             gazebo/config/o4_camera.yaml, which the rectifier (nimbus_core/rectify.py) and the depth pipeline read.
  link       diagnose the stream itself: ping loss/jitter to the CosmoStreamer, your wifi state, and for RTSP over TCP and over UDP: frame rate, frame gaps, stalls, decoder errors and lost
             RTP packets. Run this first when the picture is late or smeared.
  check      grab one live frame, run it through the rectifier with the stored calibration and save a side-by-side picture (original | rectified) to judge by eye.

usage: python3 tools_scripts/o4_calibrate.py link      [--url URL] [--seconds 10] [--transport tcp|udp]
       python3 tools_scripts/o4_calibrate.py latency   [--url URL] [--seconds 30]
       python3 tools_scripts/o4_calibrate.py calibrate [--url URL] [--cols 9 --rows 6 --square-mm 25] [--views 25] [--write]   (or --images DIR for saved pictures)
       python3 tools_scripts/o4_calibrate.py check     [--url URL]
Offline self-tests (no camera): latency --selftest, calibrate --selftest."""
import argparse, glob, json, math, os, sys, threading, time
import numpy as np
import cv2

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "software", "pc"))
CFG = os.path.join(ROOT, "gazebo", "config", "o4_camera.yaml")
LAT = os.path.join(ROOT, "gazebo", "config", "o4_latency.json")
VIEWS_DIR = os.path.join(ROOT, "docs", "o4_calibration")


def default_url() -> str:
    try: return json.load(open(os.path.expanduser("~/.config/droneforge/video.json")))["target"]
    except Exception: return "rtsp://192.168.50.1:554/video"


def open_stream(url: str, transport=None):
    """Open the stream the way the GCS Video page does: RTSP with low-delay options, RTP over TCP first (UDP loses packets over Wi-Fi: smeared frames, decoder errors).
    transport='tcp'|'udp' forces one; None = the GCS default (tcp, then udp)."""
    from nimbus_gcs.video_source import open_capture, open_rtsp
    if url.lower().startswith(("rtsp://", "rtsps://")): return open_rtsp(url, transports=(transport,) if transport else None)
    return open_capture(url)


class LatestFrameReader:
    """Reads the stream in its own thread so the socket is always drained, however slow the processing is. A loop that reads a frame and then spends 300 ms on a checkerboard search lets the
    network buffer overflow: lost RTP packets, 'bad cseq', smeared or stalled pictures (this happened in the first calibration run). latest() returns the newest frame, its arrival time and a counter."""
    def __init__(self, cap):
        self.cap = cap; self.lock = threading.Lock(); self.frame = None; self.t = 0.0; self.n = 0; self._stop = threading.Event()
        self.th = threading.Thread(target=self._run, daemon=True); self.th.start()

    def _run(self):
        while not self._stop.is_set():
            ok, fr = self.cap.read()
            if ok and fr is not None:
                with self.lock: self.frame = fr; self.t = time.monotonic(); self.n += 1
            else: time.sleep(0.002)

    def latest(self):
        with self.lock: return self.frame, self.t, self.n

    def close(self):
        self._stop.set(); self.th.join(timeout=2)


def load_cfg() -> dict:
    import yaml
    return yaml.safe_load(open(CFG))


# ------------------------------------------------------------------------------------------------ latency
def latency_stats(delays_ms, warmup=2):
    d = np.array(delays_ms[warmup:], float)
    if len(d) < 5: return None
    return {"samples": int(len(d)), "mean_ms": float(d.mean()), "median_ms": float(np.median(d)), "p95_ms": float(np.percentile(d, 95)), "std_ms": float(d.std()),
            "min_ms": float(d.min()), "max_ms": float(d.max())}


def run_latency(cap, seconds, show=True, stop_after_found=None):
    """cap: an object with read() -> (ok, frame). Returns the list of per-frame delays (ms)."""
    from nimbus_gcs import video_latency as L
    delays = []; probe = L.LatencyProbe(lambda ms: delays.append(ms)); probe.start(); t0 = time.monotonic(); last_show = 0.0; frames = 0; rd = LatestFrameReader(cap); last_n = 0
    win = "O4 latency test: point the camera at this window"
    try:
        while time.monotonic() - t0 < seconds:
            if show and time.monotonic() - last_show > 0.02:
                last_show = time.monotonic(); cv2.imshow(win, L.render_code(L.clock_ms())); cv2.waitKey(1)
            frame, t_read, n = rd.latest()
            if frame is None or n == last_n: time.sleep(0.003); continue
            frames += n - last_n if last_n else 1; last_n = n; probe.feed(frame, t_read)
    finally:
        rd.close(); probe.stop()
        if show: cv2.destroyAllWindows()
    return delays, frames, probe


class DelayedQrCapture:
    """Selftest source: frames carry the PC clock in a QR code and reach read() a fixed time later (like a real stream)."""
    def __init__(self, delay_s, jitter_s=0.0, fps=30):
        from nimbus_gcs import video_latency as L
        self.L = L; self.delay = delay_s; self.jitter = jitter_s; self.dt = 1.0 / fps; self.q = []; self.next = time.monotonic(); self.rng = np.random.default_rng(1)

    def read(self):
        now = time.monotonic()
        while self.next <= now:
            ms = self.L.clock_ms(self.next); img = self.L.render_code(ms)
            canvas = np.full((480, 640, 3), 90, np.uint8); h, w = img.shape[:2]; img3 = img if img.ndim == 3 else cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            canvas[100:100 + h, 100:100 + w] = cv2.resize(img3, (w, h)) if h + 100 < 480 and w + 100 < 640 else cv2.resize(img3, (300, 300))[:380, :540]
            self.q.append((self.next + self.delay + max(-self.delay, self.rng.normal(0, self.jitter)), canvas)); self.next += self.dt
        while self.q and self.q[0][0] <= now:
            return True, self.q.pop(0)[1]
        time.sleep(0.003); return False, None


def cmd_latency(a):
    if a.selftest:
        cap = DelayedQrCapture(0.180, 0.020); delays, frames, probe = run_latency(cap, a.seconds if a.seconds != 30 else 12, show=False)
        st = latency_stats(delays, 2); print("selftest (true delay 180 ms, jitter 20 ms):", json.dumps({k: round(v, 1) if isinstance(v, float) else v for k, v in (st or {}).items()})); return 0 if st and abs(st["mean_ms"] - 180) < 60 else 1
    url = a.url or default_url(); print(f"opening {url} (transport {a.transport or 'tcp first'}) ...")
    cap = open_stream(url, a.transport)
    if not cap.isOpened(): print("could not open the stream (is CosmoStreamer's RTSP server on and the PC on the drone's network?)"); return 1
    print(f"A window with a QR code opens. Point the O4 camera at it, filling a good part of the picture, and keep it steady for {a.seconds:.0f} s.")
    delays, frames, probe = run_latency(cap, a.seconds, show=True); cap.release()
    st = latency_stats(delays, 2); print(f"frames read {frames}, code decoded in {probe.found} of {probe.attempts} attempts")
    if not st: print("too few readings: the code was not seen often enough. Move the camera closer, reduce glare, make the QR window larger."); return 1
    print("camera-to-PC delay: mean %.0f ms, median %.0f, p95 %.0f, jitter (std) %.0f, range %.0f..%.0f (%d readings)" % (st["mean_ms"], st["median_ms"], st["p95_ms"], st["std_ms"], st["min_ms"], st["max_ms"], st["samples"]))
    st.update(url=url, measured=time.strftime("%Y-%m-%d %H:%M:%S"), note="includes camera exposure and this screen's refresh (a few tens of ms)")
    json.dump(st, open(LAT, "w"), indent=1); print("written", LAT, "(the simulation reads it: --cam-latency / --cam-jitter default to these)"); return 0


# ------------------------------------------------------------------------------------------------ calibration
def board_points(cols, rows, square_m):
    p = np.zeros((cols * rows, 3), np.float64); p[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_m; return p


def find_board(gray, cols, rows):
    flags = cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
    try: ok, c = cv2.findChessboardCornersSB(gray, (cols, rows), flags)
    except Exception: ok, c = False, None
    if not ok:
        ok, c = cv2.findChessboardCorners(gray, (cols, rows), cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK)
        if ok: c = cv2.cornerSubPix(gray, c, (7, 7), (-1, -1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001))
    return (c.reshape(-1, 2).astype(np.float64) if ok else None)


def view_signature(c, size):
    """Where the board is, how big and how tilted: used to accept only views that add something."""
    w, h = size; cen = c.mean(0) / np.array([w, h]); span = (c.max(0) - c.min(0)) / np.array([w, h]); skew = float(np.hypot(*(c[-1] - c[0])) / max(1.0, np.hypot(*(c[len(c) // 2] - c[0])) * 2))
    return np.array([cen[0], cen[1], span.mean() * 1.5, float(span[0] / max(1e-3, span[1])) * 0.3 + skew * 0.2])


def _fe(name):
    """A fisheye calibration flag: cv2.fisheye.<NAME> in OpenCV 4, cv2.<NAME> in OpenCV 5."""
    return getattr(cv2.fisheye, name, None) if hasattr(cv2.fisheye, name) else getattr(cv2, name)


def view_usable(c, P, K0, D0, fisheye=True):
    """A view the fit can start from: the board plane maps to the picture with a well-conditioned homography (finite, both axes alike in scale, board in front of the camera).
    A mis-detected board, a board seen edge-on or points the lens model cannot place would make the whole calibration fail with an opaque OpenCV error."""
    try:
        n = (cv2.fisheye.undistortPoints(c.reshape(-1, 1, 2), K0, D0.reshape(-1, 1)[:4]) if fisheye else cv2.undistortPoints(c.reshape(-1, 1, 2), K0, D0)).reshape(-1, 2)
        H, _ = cv2.findHomography(P[:, :2], n, 0)
        if H is None or not np.isfinite(H).all(): return False
        h1, h2 = H[:, 0], H[:, 1]; n1, n2 = np.linalg.norm(h1), np.linalg.norm(h2)
        return bool(n1 > 1e-6 and n2 > 1e-6 and 0.4 < n1 / n2 < 2.5 and abs(np.dot(h1, h2)) / (n1 * n2) < 0.6 and abs(H[2, 2]) > 1e-9)
    except Exception:
        return False


def calibrate_fisheye(corner_sets, size, cols, rows, square_m, K0=None):
    """Equidistant fisheye fit (cv2.fisheye). Returns dict(K, D(4), rms, per_view). Array shapes: OpenCV 5.0 needs (1,N,3)/(1,N,2); older versions take (N,1,..): both are tried."""
    P = board_points(cols, rows, square_m)
    K = np.array(K0, np.float64) if K0 is not None else np.array([[size[0] / 2.0, 0, size[0] / 2.0], [0, size[0] / 2.0, size[1] / 2.0], [0, 0, 1.0]]); D = np.zeros((4, 1))
    flags = _fe("CALIB_RECOMPUTE_EXTRINSIC") | _fe("CALIB_FIX_SKEW") | (_fe("CALIB_USE_INTRINSIC_GUESS") if hasattr(cv2.fisheye, "CALIB_USE_INTRINSIC_GUESS") or hasattr(cv2, "CALIB_USE_INTRINSIC_GUESS") else 0)
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-9); last = None
    good = [i for i, c in enumerate(corner_sets) if view_usable(c, P, K, D)]
    if len(good) < len(corner_sets): print(f"screened out {len(corner_sets) - len(good)} view(s) with a degenerate pose; {len(good)} left")
    if len(good) < 8: raise RuntimeError(f"only {len(good)} usable views: tilt the board more, bring it closer, and cover the whole picture, then repeat")
    corner_sets = [corner_sets[i] for i in good]; keep = list(range(len(corner_sets)))
    for shape in ("1N", "N1"):
        mk_o = (lambda: P.reshape(1, -1, 3)) if shape == "1N" else (lambda: P.reshape(-1, 1, 3)); mk_i = (lambda c: c.reshape(1, -1, 2)) if shape == "1N" else (lambda c: c.reshape(-1, 1, 2))
        keep = list(range(len(corner_sets))); ok = False
        for attempt in range(6):                                          # the fit can reject an ill-conditioned view: drop it and retry
            try:
                rms, K1, D1, rv, tv = cv2.fisheye.calibrate([mk_o() for _ in keep], [mk_i(corner_sets[i]) for i in keep], size, K.copy(), D.copy(), flags=flags + (_fe("CALIB_CHECK_COND") if attempt < 3 else 0), criteria=crit); ok = True; break
            except cv2.error as e:
                last = e; import re; m = re.search(r"input array (\d+)", str(e)) or re.search(r"image (\d+)", str(e))
                if m and int(m.group(1)) < len(keep): keep.pop(int(m.group(1)))
                else: break
        if ok: break
    else:
        raise last
    per = []
    for k, i in enumerate(keep):
        proj, _ = cv2.fisheye.projectPoints(P.reshape(-1, 1, 3), np.asarray(rv[k]).reshape(3, 1), np.asarray(tv[k]).reshape(3, 1), K1, D1.reshape(4, 1)); per.append(float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - corner_sets[i]) ** 2, axis=1)))))
    return {"K": K1, "D": D1.reshape(-1), "rms": float(rms), "per_view": per, "kept": keep}


def calibrate_pinhole(corner_sets, size, cols, rows, square_m):
    obj = [board_points(cols, rows, square_m).astype(np.float32) for _ in corner_sets]; img = [c.astype(np.float32).reshape(-1, 1, 2) for c in corner_sets]
    rms, K, D, rv, tv = cv2.calibrateCamera(obj, img, size, None, None, flags=cv2.CALIB_FIX_K3)
    return {"K": K, "D": D.reshape(-1)[:5], "rms": float(rms)}


def field_of_view(K, D, size, fisheye=True):
    """Horizontal, vertical and diagonal field of view in degrees of the calibrated model over the full picture."""
    w, h = size
    pts = np.array([[[0, h / 2]], [[w - 1, h / 2]], [[w / 2, 0]], [[w / 2, h - 1]], [[0, 0]], [[w - 1, h - 1]]], np.float64)
    n = (cv2.fisheye.undistortPoints(pts, K, D.reshape(-1, 1)[:4]) if fisheye else cv2.undistortPoints(pts, K, D)).reshape(-1, 2)
    hfov = math.degrees(math.atan(n[1, 0]) - math.atan(n[0, 0])); vfov = math.degrees(math.atan(n[3, 1]) - math.atan(n[2, 1]))
    r = np.column_stack([n, np.ones(len(n))]); r /= np.linalg.norm(r, axis=1)[:, None]; dfov = math.degrees(math.acos(float(np.clip(r[4] @ r[5], -1, 1))))
    if not np.isfinite(n).all() or dfov < max(hfov, vfov) - 1.0: dfov = float("nan")          # the corners lie outside what the fitted lens can describe (no calibration views reached them)
    return hfov, vfov, dfov


def write_calibration(res, size, fisheye):
    K, D = res["K"], res["D"]
    entry = "calibration: {fx: %.3f, fy: %.3f, cx: %.3f, cy: %.3f, dist: [%s], model: %s, size: [%d, %d], rms_px: %.3f, calibrated: \"%s\"}" % (
        K[0, 0], K[1, 1], K[0, 2], K[1, 2], ", ".join("%.6f" % v for v in D), "fisheye" if fisheye else "pinhole", size[0], size[1], res["rms"], time.strftime("%Y-%m-%d"))
    lines = open(CFG).read().split("\n"); out = []; done = False
    for ln in lines:
        if ln.startswith("calibration:") and not done: out.append(entry); done = True
        else: out.append(ln)
    if not done: out.append(entry)
    open(CFG, "w").write("\n".join(out)); return entry


def report_and_store(corner_sets, size, a, variant_lens):
    fisheye = variant_lens == "equidistant"; cfgd = load_cfg(); v = cfgd["variants"][cfgd["variant"]]
    from nimbus_core.rectify import nominal_K
    K0 = nominal_K(size[0], size[1], v["diagonal_fov_deg"], variant_lens); sq = a.square_mm / 1000.0
    res = calibrate_fisheye(corner_sets, size, a.cols, a.rows, sq, K0) if fisheye else calibrate_pinhole(corner_sets, size, a.cols, a.rows, sq)
    if fisheye and res["per_view"]:                                      # drop views that fit badly (a mis-detected board) and fit again once
        med = float(np.median(res["per_view"])); bad = [res["kept"][i] for i, e in enumerate(res["per_view"]) if e > max(3 * med, 1.0)]
        if bad and len(corner_sets) - len(bad) >= 8: corner_sets = [c for i, c in enumerate(corner_sets) if i not in bad]; print(f"dropped {len(bad)} badly fitting view(s), refitting on {len(corner_sets)}"); res = calibrate_fisheye(corner_sets, size, a.cols, a.rows, sq, K0)
    h, vv, d = field_of_view(res["K"], res["D"], size, fisheye)
    print(f"model {'equidistant fisheye' if fisheye else 'pinhole'}: fx {res['K'][0,0]:.1f} fy {res['K'][1,1]:.1f} cx {res['K'][0,2]:.1f} cy {res['K'][1,2]:.1f}  distortion {np.round(res['D'], 4).tolist()}")
    print(f"reprojection error {res['rms']:.2f} px RMS  (good: below about 0.8 px; above 2 px the views or the board are bad)")
    dtxt = f"diagonal {d:.0f} deg" if d == d else "diagonal NOT determined (the views did not reach the corners)"
    print(f"REAL field of view of this stream: horizontal {h:.0f} deg, vertical {vv:.0f} deg, {dtxt}   (the YAML assumed {v['diagonal_fov_deg']:.0f} deg diagonal for the {cfgd['variant']} variant)")
    allp = np.vstack(corner_sets); w_, h_ = size; fx_lo, fx_hi, fy_lo, fy_hi = (np.percentile(allp[:, 0], 1) / w_, np.percentile(allp[:, 0], 99) / w_, np.percentile(allp[:, 1], 1) / h_, np.percentile(allp[:, 1], 99) / h_)
    print("the board corners reached x %.0f..%.0f %% and y %.0f..%.0f %% of the picture" % (100 * fx_lo, 100 * fx_hi, 100 * fy_lo, 100 * fy_hi))
    if fx_lo > 0.12 or fx_hi < 0.88 or fy_lo > 0.15 or fy_hi < 0.85:
        print("WARNING: the views do not reach the picture edges, so the lens distortion there (and the field of view) is EXTRAPOLATED and may be wrong. Take more views with the board near each edge and corner, then repeat.")
    ok = res["rms"] < 2.0
    if a.write and ok: print("written to", CFG, ":", write_calibration(res, size, fisheye))
    elif a.write: print("NOT written: the error is too high to trust")
    else: print("(not written: add --write to store it)")
    return res, ok


def cmd_calibrate(a):
    cfgd = load_cfg(); v = cfgd["variants"][cfgd["variant"]]; lens = v.get("lens", "rectilinear")
    if a.selftest: return selftest_calibrate(a, lens)
    corner_sets = []; size = None
    if a.images:
        for f in sorted(glob.glob(os.path.join(a.images, "*"))):
            im = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
            if im is None: continue
            size = (im.shape[1], im.shape[0]); c = find_board(im, a.cols, a.rows)
            if c is not None: corner_sets.append(c)
        print(f"{len(corner_sets)} usable pictures from {a.images}")
    else:
        url = a.url or default_url(); print(f"opening {url} (transport {a.transport or 'tcp first'}) ...")
        cap = open_stream(url, a.transport)
        if not cap.isOpened(): print("could not open the stream"); return 1
        os.makedirs(VIEWS_DIR, exist_ok=True); sigs = []; t0 = time.monotonic(); last = 0.0; win = "O4 calibration: show the checkerboard"; rd = LatestFrameReader(cap); last_n = 0
        print(f"Hold the printed {a.cols+1}x{a.rows+1}-square checkerboard ({a.square_mm:.0f} mm squares) FLAT (on a rigid board) in front of the camera. Move it: centre, every corner of the picture, close, far, tilted both ways.")
        print(f"The tool keeps a view when the board is found and clearly different from the kept ones; target {a.views} views. Press q in the window to finish early.")
        while len(corner_sets) < a.views and time.monotonic() - t0 < a.timeout:
            fr, _t, n = rd.latest()
            if fr is None or n == last_n: time.sleep(0.005); cv2.waitKey(1); continue
            last_n = n; size = (fr.shape[1], fr.shape[0])
            if time.monotonic() - last < 0.4: continue                            # the reader thread keeps draining the stream while this (slow) step runs
            gray = cv2.cvtColor(fr, cv2.COLOR_BGR2GRAY)
            last = time.monotonic(); c = find_board(gray, a.cols, a.rows); vis = fr.copy()
            if c is not None:
                sg = view_signature(c, size); new = all(np.linalg.norm(sg - s) > 0.12 for s in sigs)
                cv2.drawChessboardCorners(vis, (a.cols, a.rows), c.reshape(-1, 1, 2).astype(np.float32), True)
                if new: sigs.append(sg); corner_sets.append(c); cv2.imwrite(os.path.join(VIEWS_DIR, f"view_{len(corner_sets):02d}.png"), fr)
                cv2.putText(vis, "KEPT" if new else "found, move it", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0) if new else (0, 200, 255), 2)
            cv2.putText(vis, f"views {len(corner_sets)}/{a.views}", (20, vis.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
            cv2.imshow(win, cv2.resize(vis, None, fx=0.6, fy=0.6) if vis.shape[1] > 1100 else vis)
            if cv2.waitKey(1) & 0xFF == ord("q"): break
        rd.close(); cap.release(); cv2.destroyAllWindows()
    if len(corner_sets) < 10: print(f"only {len(corner_sets)} views: need at least 10 (20+ is better). Nothing computed."); return 1
    xs = np.array([c.mean(0) for c in corner_sets]); w, h = size
    print("coverage of the picture by the board centres: x %.0f..%.0f %%, y %.0f..%.0f %%" % (100 * xs[:, 0].min() / w, 100 * xs[:, 0].max() / w, 100 * xs[:, 1].min() / h, 100 * xs[:, 1].max() / h))
    res, ok = report_and_store(corner_sets, size, a, lens); return 0 if ok else 1


def selftest_calibrate(a, lens):
    """No camera: synthesise corner sets from a known fisheye model (with 0.3 px noise), calibrate, compare."""
    size = (1280, 720); Kt = np.array([[560.0, 0, 640.0], [0, 560.0, 360.0], [0, 0, 1.0]]); Dt = np.array([-0.02, 0.008, -0.004, 0.001]).reshape(4, 1); rng = np.random.default_rng(3); sq = a.square_mm / 1000.0
    obj = board_points(a.cols, a.rows, sq); sets = []
    while len(sets) < 24:
        rv = rng.normal(0, 0.35, 3); tv = np.array([rng.uniform(-0.25, 0.25), rng.uniform(-0.15, 0.15), rng.uniform(0.25, 0.8)]) - np.array([(a.cols - 1) * sq / 2, (a.rows - 1) * sq / 2, 0])
        R, _ = cv2.Rodrigues(rv); pc = (R @ obj.T).T + tv
        if (pc[:, 2] < 0.1).any(): continue
        px, _ = cv2.fisheye.projectPoints(obj.reshape(-1, 1, 3), rv, tv, Kt, Dt); px = px.reshape(-1, 2) + rng.normal(0, 0.3, (len(obj), 2))
        if (px[:, 0] < 5).any() or (px[:, 0] > size[0] - 5).any() or (px[:, 1] < 5).any() or (px[:, 1] > size[1] - 5).any(): continue
        sets.append(px)
    r = calibrate_fisheye(sets, size, a.cols, a.rows, sq, np.array([[500.0, 0, 640.0], [0, 500.0, 360.0], [0, 0, 1.0]]))
    ht, vt, dt = field_of_view(Kt, Dt, size, True); h, v, d = field_of_view(r["K"], r["D"], size, True)
    print("selftest: true fx %.1f fy %.1f cx %.1f cy %.1f | fitted fx %.1f fy %.1f cx %.1f cy %.1f | rms %.2f px | FOV true %.1f/%.1f/%.1f fitted %.1f/%.1f/%.1f" % (
        Kt[0, 0], Kt[1, 1], Kt[0, 2], Kt[1, 2], r["K"][0, 0], r["K"][1, 1], r["K"][0, 2], r["K"][1, 2], r["rms"], ht, vt, dt, h, v, d))
    return 0 if abs(r["K"][0, 0] - 560) < 8 and abs(h - ht) < 2.0 and r["rms"] < 1.0 else 1


# ------------------------------------------------------------------------------------------------ stream health
def gap_stats(times):
    """Frame arrival statistics from a list of arrival times (s)."""
    t = np.array(times, float)
    if len(t) < 5: return None
    g = np.diff(t) * 1000.0
    return {"frames": int(len(t)), "fps": float((len(t) - 1) / (t[-1] - t[0])), "gap_p50_ms": float(np.percentile(g, 50)), "gap_p95_ms": float(np.percentile(g, 95)), "gap_max_ms": float(g.max()),
            "stalls_100ms": int((g > 100).sum()), "stalls_250ms": int((g > 250).sum())}


def parse_ping(text):
    import re
    loss = re.search(r"([\d.]+)% packet loss", text); rtt = re.search(r"= ([\d.]+)/([\d.]+)/([\d.]+)/([\d.]+) ms", text)
    return {"loss_pct": float(loss.group(1)) if loss else None, "min": float(rtt.group(1)) if rtt else None, "avg": float(rtt.group(2)) if rtt else None, "max": float(rtt.group(3)) if rtt else None, "mdev": float(rtt.group(4)) if rtt else None}


def count_errors(text):
    return {"decode_errors": text.count("error while decoding"), "rtp_seq_errors": text.count("bad cseq"), "missed_pkts": text.count("missed") + text.count("RTP: missed")}


def read_with_stderr_capture(cap, seconds):
    """Read frames for `seconds`; returns (arrival times, the text FFmpeg printed meanwhile). FFmpeg writes its warnings to the process's stderr, so fd 2 is redirected to a file for the duration."""
    import tempfile
    tf = tempfile.TemporaryFile(mode="w+b"); saved = os.dup(2); sys.stderr.flush(); os.dup2(tf.fileno(), 2); times = []
    try:
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            ok, fr = cap.read()
            if ok and fr is not None: times.append(time.monotonic())
            else: time.sleep(0.003)
    finally:
        sys.stderr.flush(); os.dup2(saved, 2); os.close(saved)
    tf.seek(0); txt = tf.read().decode("utf-8", "replace"); tf.close(); return times, txt


def cmd_link(a):
    import subprocess, re
    url = a.url or default_url(); host = re.sub(r"^[a-z]+://", "", url).split("/")[0].split(":")[0]
    print(f"stream {url}   (host {host})")
    owner = os.environ.get("OPENCV_FFMPEG_CAPTURE_OPTIONS"); print("OPENCV_FFMPEG_CAPTURE_OPTIONS in this shell:", owner if owner else "not set (the tool uses the GCS low-delay options)")
    try:
        w = subprocess.run(["nmcli", "-t", "-f", "ACTIVE,SSID,SIGNAL,RATE,CHAN,FREQ", "dev", "wifi"], capture_output=True, text=True, timeout=5).stdout
        for ln in w.splitlines():
            if ln.startswith("yes"): print("laptop wifi:", ln[4:].replace(":", "  "))
        ifs = subprocess.run(["iw", "dev"], capture_output=True, text=True, timeout=5).stdout; m = re.search(r"Interface (\S+)", ifs)
        if m:
            ps = subprocess.run(["iw", "dev", m.group(1), "get", "power_save"], capture_output=True, text=True, timeout=5).stdout.strip(); print(f"{m.group(1)}: {ps}   (power save ON adds delay and jitter: sudo iw dev {m.group(1)} set power_save off)")
    except Exception as e: print("wifi info not available:", e)
    print(f"\nping {host} (30 packets, 0.2 s apart) ...")
    try:
        pg = parse_ping(subprocess.run(["ping", "-c", "30", "-i", "0.2", "-W", "2", host], capture_output=True, text=True, timeout=30).stdout)
        print("   loss %s %%, rtt min/avg/max %s/%s/%s ms, jitter %s ms" % (pg["loss_pct"], pg["min"], pg["avg"], pg["max"], pg["mdev"]))
        if pg["loss_pct"] == 100.0: print("   the host does not answer: is this laptop on the CosmoStreamer network (not the incubation wifi)?"); return 1
    except Exception as e: print("   ping failed:", e)
    verdict = {}
    for tr in ([a.transport] if a.transport else ["tcp", "udp"]):
        print(f"\nRTSP over {tr.upper()}: reading for {a.seconds:.0f} s ...")
        t0 = time.monotonic(); cap = open_stream(url, tr)
        if not cap.isOpened(): print("   could not open"); continue
        opened = time.monotonic() - t0; times, txt = read_with_stderr_capture(cap, a.seconds); cap.release()
        st = gap_stats(times); er = count_errors(txt)
        if not st: print("   almost no frames received", er); verdict[tr] = None; continue
        print("   opened in %.1f s; %d frames, %.1f fps; gap between frames p50 %.0f ms, p95 %.0f ms, max %.0f ms; stalls >100 ms: %d, >250 ms: %d" % (opened, st["frames"], st["fps"], st["gap_p50_ms"], st["gap_p95_ms"], st["gap_max_ms"], st["stalls_100ms"], st["stalls_250ms"]))
        print("   decoder errors %d, RTP sequence errors (lost/reordered packets) %d" % (er["decode_errors"], er["rtp_seq_errors"])); verdict[tr] = (st, er)
    print("\nreading this:")
    print(" - decoder / RTP sequence errors with UDP and few with TCP: packets are lost on the air; TCP hides it (costs a little latency). The GCS uses TCP first.")
    print(" - errors with BOTH: the radio link is poor (signal, channel, distance, interference) or the stream bitrate is too high for it: lower the bitrate/resolution in CosmoStreamer.")
    print(" - fps well below the camera's, or p95 gaps >100 ms with no errors: the laptop is the limit (CPU load, wifi power save) or the stream is being throttled.")
    print(" - ping jitter high or loss >1 %: the wifi link itself; move closer to the CosmoStreamer, avoid a busy channel (5 GHz if offered).")
    return 0


# ------------------------------------------------------------------------------------------------ rectifier check
def cmd_check(a):
    from nimbus_core.rectify import Rectifier
    cfgd = load_cfg(); v = cfgd["variants"][cfgd["variant"]]; cal = cfgd.get("calibration")
    url = a.url or default_url()
    cap = open_stream(url, a.transport)
    if not cap.isOpened(): print("could not open the stream"); return 1
    fr = None
    for _ in range(60):
        ok, f = cap.read()
        if ok and f is not None: fr = f
        if fr is not None and _ > 10: break
    cap.release()
    if fr is None: print("no frame received"); return 1
    h, w = fr.shape[:2]; K = None; dist = None
    if cal:
        K = [[cal["fx"], 0, cal["cx"]], [0, cal["fy"], cal["cy"]], [0, 0, 1]]
        if cal.get("size") and tuple(cal["size"]) != (w, h): s = w / cal["size"][0]; K = [[cal["fx"] * s, 0, cal["cx"] * s], [0, cal["fy"] * s, cal["cy"] * s], [0, 0, 1]]; print(f"calibration was made at {cal['size']}, scaled to {w}x{h}")
        dist = cal.get("dist")
    print(f"frame {w}x{h}; lens {v.get('lens')}; calibration {'YES (rms %.2f px)' % cal.get('rms_px', float('nan')) if cal else 'NO (nominal lens: the result is only approximate)'}")
    rect = Rectifier(w, h, K, dist, v["diagonal_fov_deg"], lens=v.get("lens", "rectilinear"))(fr)
    side = np.hstack([cv2.resize(fr, (int(fr.shape[1] * 480 / fr.shape[0]), 480)), rect]); out = os.path.join(VIEWS_DIR, "rectifier_check.png"); os.makedirs(VIEWS_DIR, exist_ok=True); cv2.imwrite(out, side)
    print("saved", out, "(left: the stream as received; right: what the depth model will see: straight lines should be straight, no black borders)"); return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter); sp = ap.add_subparsers(dest="cmd", required=True)
    p = sp.add_parser("link"); p.add_argument("--url"); p.add_argument("--seconds", type=float, default=10.0); p.add_argument("--transport", choices=("tcp", "udp"))
    p = sp.add_parser("latency"); p.add_argument("--url"); p.add_argument("--transport", choices=("tcp", "udp")); p.add_argument("--seconds", type=float, default=30.0); p.add_argument("--selftest", action="store_true")
    p = sp.add_parser("calibrate"); p.add_argument("--url"); p.add_argument("--transport", choices=("tcp", "udp")); p.add_argument("--cols", type=int, default=9); p.add_argument("--rows", type=int, default=6); p.add_argument("--square-mm", dest="square_mm", type=float, default=25.0)
    p.add_argument("--views", type=int, default=25); p.add_argument("--timeout", type=float, default=300.0); p.add_argument("--images"); p.add_argument("--write", action="store_true"); p.add_argument("--selftest", action="store_true")
    p = sp.add_parser("check"); p.add_argument("--url"); p.add_argument("--transport", choices=("tcp", "udp"))
    a = ap.parse_args(); sys.exit({"link": cmd_link, "latency": cmd_latency, "calibrate": cmd_calibrate, "check": cmd_check}[a.cmd](a))


if __name__ == "__main__":
    main()
