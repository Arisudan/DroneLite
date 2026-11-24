import importlib.util, os, shutil, sys, tempfile, time
import numpy as np
import cv2
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
spec = importlib.util.spec_from_file_location("o4_calibrate", os.path.join(ROOT, "tools_scripts", "o4_calibrate.py")); o4 = importlib.util.module_from_spec(spec); spec.loader.exec_module(o4)
COLS, ROWS = 9, 6


def _board_image(w=1280, h=720, seed=0):
    """A flat checkerboard (COLS+1 x ROWS+1 squares) seen through a random perspective: a stand-in for a picture of the printed board."""
    sq = 60; pat = np.zeros(((ROWS + 3) * sq, (COLS + 3) * sq), np.uint8)
    for r in range(ROWS + 3):
        for c in range(COLS + 3):
            if (r + c) % 2 == 0: pat[r * sq:(r + 1) * sq, c * sq:(c + 1) * sq] = 255
    rng = np.random.default_rng(seed); ph, pw = pat.shape
    src = np.float32([[0, 0], [pw, 0], [pw, ph], [0, ph]]); s = rng.uniform(0.45, 0.8); cx, cy = w / 2 + rng.uniform(-250, 250), h / 2 + rng.uniform(-130, 130)
    dst = np.float32([[cx - pw * s / 2 + rng.uniform(-50, 50), cy - ph * s / 2 + rng.uniform(-30, 30)], [cx + pw * s / 2 + rng.uniform(-50, 50), cy - ph * s / 2 + rng.uniform(-30, 30)],
                      [cx + pw * s / 2 + rng.uniform(-50, 50), cy + ph * s / 2 + rng.uniform(-30, 30)], [cx - pw * s / 2 + rng.uniform(-50, 50), cy + ph * s / 2 + rng.uniform(-30, 30)]])
    img = cv2.warpPerspective(pat, cv2.getPerspectiveTransform(src, dst), (w, h), borderValue=128); return cv2.GaussianBlur(img, (3, 3), 0.8)


def test_board_is_detected_in_a_rendered_picture():
    found = sum(o4.find_board(_board_image(seed=i), COLS, ROWS) is not None for i in range(6)); assert found >= 5


def test_fisheye_fit_recovers_a_known_lens():
    class A: cols, rows, square_mm = COLS, ROWS, 25.0
    assert o4.selftest_calibrate(A, "equidistant") == 0


def test_latency_statistics_and_selftest_source():
    st = o4.latency_stats([300.0] * 2 + list(np.random.default_rng(0).normal(180, 15, 200)), warmup=2)
    assert abs(st["mean_ms"] - 180) < 5 and 10 < st["std_ms"] < 20 and st["samples"] == 200
    assert o4.latency_stats([100.0] * 4) is None                                       # too few readings


def test_write_calibration_stores_it_where_the_pipeline_reads_it():
    tmp = tempfile.mkdtemp(); path = os.path.join(tmp, "o4_camera.yaml"); shutil.copy(os.path.join(ROOT, "gazebo", "config", "o4_camera.yaml"), path); old = o4.CFG; o4.CFG = path
    try:
        res = {"K": np.array([[540.0, 0, 641.0], [0, 541.0, 358.0], [0, 0, 1.0]]), "D": np.array([-0.02, 0.01, -0.004, 0.001]), "rms": 0.55}
        o4.write_calibration(res, (1280, 720), True); import yaml; cal = yaml.safe_load(open(path))["calibration"]
        assert abs(cal["fx"] - 540.0) < 1e-6 and cal["dist"] == [-0.02, 0.01, -0.004, 0.001] and cal["size"] == [1280, 720] and cal["model"] == "fisheye"
        from nimbus_core.rectify import Rectifier                                       # and the rectifier accepts exactly this entry
        K = [[cal["fx"], 0, cal["cx"]], [0, cal["fy"], cal["cy"]], [0, 0, 1]]; r = Rectifier(1280, 720, K, cal["dist"], 155.0, lens="equidistant"); assert r(np.zeros((720, 1280, 3), np.uint8)).shape == (480, 640, 3)
    finally: o4.CFG = old; shutil.rmtree(tmp)


def test_stream_health_helpers():
    t = np.cumsum(np.r_[0.0, np.full(99, 0.033)]); t[50:] += 0.4                       # 30 fps with one 400 ms stall
    st = o4.gap_stats(t.tolist()); assert 26 < st["fps"] < 28 and st["stalls_250ms"] == 1 and st["gap_max_ms"] > 390
    pg = o4.parse_ping("30 packets transmitted, 29 received, 3.3% packet loss, time 6000ms\nrtt min/avg/max/mdev = 2.1/9.8/80.5/14.2 ms")
    assert pg["loss_pct"] == 3.3 and pg["avg"] == 9.8 and pg["mdev"] == 14.2
    er = o4.count_errors("[h264] error while decoding MB 1 2\n[rtsp] RTP: PT=60: bad cseq 1 expected=2\n[h264] error while decoding MB 3 4\n"); assert er["decode_errors"] == 2 and er["rtp_seq_errors"] == 1


def test_stderr_capture_sees_ffmpeg_style_output():
    class Cap:
        def __init__(self): self.n = 0
        def read(self):
            self.n += 1; os.write(2, b"[h264 @ 0x1] error while decoding MB 5 5\n"); time.sleep(0.01); return True, np.zeros((4, 4, 3), np.uint8)
    times, txt = o4.read_with_stderr_capture(Cap(), 0.3); assert len(times) > 10 and o4.count_errors(txt)["decode_errors"] == len(times)


def _fisheye_board_image(K, D, size, rvec, tvec, sq=0.025, seed=0):
    """A checkerboard (COLS+1 x ROWS+1 squares plus a white margin) seen through a fisheye lens: every pixel is traced to its ray, intersected with the board plane, and given the board's colour there."""
    w, h = size; gx, gy = np.meshgrid(np.arange(w, dtype=np.float64), np.arange(h, dtype=np.float64))
    n = cv2.fisheye.undistortPoints(np.stack([gx, gy], -1).reshape(-1, 1, 2), K, D.reshape(-1, 1)).reshape(-1, 2)          # x/z, y/z of every pixel's ray
    rays = np.column_stack([n, np.ones(len(n))]); R, _ = cv2.Rodrigues(rvec); C = -R.T @ np.asarray(tvec, float).reshape(3); dirs = rays @ R                      # camera centre and ray directions in board coordinates
    with np.errstate(divide="ignore", invalid="ignore"): lam = -C[2] / dirs[:, 2]
    X = C[0] + lam * dirs[:, 0]; Y = C[1] + lam * dirs[:, 1]; ok = lam > 0
    bx, by = X / sq + 1.0, Y / sq + 1.0                                        # board squares; the inner corners are at integer positions 1..COLS and 1..ROWS
    inside = ok & (bx > -1) & (bx < COLS + 2) & (by > -1) & (by < ROWS + 2); on = inside & (bx > 0) & (bx < COLS + 1) & (by > 0) & (by < ROWS + 1)
    parity = (np.floor(bx).astype(int) + np.floor(by).astype(int)) % 2
    img = np.full(h * w, 128, np.uint8); img[inside] = 255; img[on] = np.where(parity[on] == 0, 20, 235).astype(np.uint8)
    return cv2.GaussianBlur(img.reshape(h, w), (3, 3), 0.9)


def test_fisheye_board_pictures_end_to_end():
    """Render boards through a known fisheye lens, detect them, calibrate from the detections, and compare with the lens that made the pictures."""
    size = (1280, 720); Kt = np.array([[540.0, 0, 641.0], [0, 540.0, 358.0], [0, 0, 1.0]]); Dt = np.array([-0.02, 0.008, -0.004, 0.001]); rng = np.random.default_rng(5); sets = []; tries = 0
    while len(sets) < 22 and tries < 60:
        tries += 1; rv = rng.normal(0, 0.45, 3) * np.array([1.0, 1.0, 0.5]); tv = np.array([rng.uniform(-0.18, 0.18), rng.uniform(-0.10, 0.10), rng.uniform(0.28, 0.55)]) - np.array([0.1125, 0.075, 0])
        c = o4.find_board(_fisheye_board_image(Kt, Dt, size, rv, tv), COLS, ROWS)
        if c is not None: sets.append(c)
    assert len(sets) >= 14, f"only {len(sets)} boards detected"
    from nimbus_core.rectify import nominal_K
    r = o4.calibrate_fisheye(sets, size, COLS, ROWS, 0.025, nominal_K(1280, 720, 155.0, "equidistant"))
    assert r["rms"] < 1.0 and abs(r["K"][0, 0] - 540.0) < 12 and abs(r["K"][0, 2] - 641.0) < 6 and abs(r["K"][1, 2] - 358.0) < 6, (r["rms"], r["K"].tolist())
