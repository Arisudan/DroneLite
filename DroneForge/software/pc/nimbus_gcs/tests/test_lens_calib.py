"""lens_calib: the solver recovers a known fisheye lens; the collector takes only still, varied views from real (rendered) pictures; save keeps the YAML comments."""
import os
import shutil

import cv2
import numpy as np

from nimbus_gcs import lens_calib as L

W, H = 1280, 720
K_TRUE = np.array([[420.0, 0, 642.0], [0, 421.0, 357.0], [0, 0, 1]]); D_TRUE = np.array([[0.05], [-0.02], [0.01], [-0.003]])


def _poses(n=25, seed=1):
    rng = np.random.default_rng(seed)
    for _ in range(n):
        yield rng.uniform(-0.5, 0.5, 3), np.array([rng.uniform(-6, -2), rng.uniform(-4, -1), rng.uniform(5, 10)])


def test_solver_recovers_a_known_fisheye_lens():
    cols, rows = 9, 6; obj = np.zeros((1, cols * rows, 3)); obj[0, :, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    views = []
    for rv, tv in _poses():
        img, _ = cv2.fisheye.projectPoints(obj, rv.reshape(3, 1), tv.reshape(3, 1), K_TRUE, D_TRUE)
        views.append((img.reshape(-1, 2) + np.random.default_rng(0).normal(0, 0.2, (cols * rows, 2))).astype(np.float32))
    cal = L.solve(views, (cols, rows), (W, H), "fisheye")
    assert abs(cal["fx"] - 420) < 3 and abs(cal["fy"] - 421) < 3 and abs(cal["cx"] - 642) < 3 and abs(cal["cy"] - 357) < 3, cal
    assert cal["rms_px"] < 0.5 and len(cal["dist"]) == 4 and cal["model"] == "fisheye"


def _render(pose_px, angle_deg, scale, tilt):
    """The printable board seen by a pinhole camera: a perspective warp into a 1280x720 grey picture."""
    b = L.board_image(); h, w = b.shape; src = np.float32([[0, 0], [w, 0], [w, h], [0, h]])
    cx, cy = pose_px; hw, hh = w * scale / 2, h * scale / 2; a = np.radians(angle_deg)
    pts = np.array([[-hw, -hh], [hw, -hh], [hw, hh], [-hw, hh]]); pts[:, 0] *= 1 + tilt * np.sign(pts[:, 1]) * 0.15
    R = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]]); dst = (pts @ R.T + [cx, cy]).astype(np.float32)
    out = cv2.warpPerspective(b, cv2.getPerspectiveTransform(src, dst), (W, H), borderValue=200)
    return cv2.cvtColor(out, cv2.COLOR_GRAY2BGR)


def test_collector_takes_only_still_varied_views():
    c = L.LensCalibrator(lambda: (None, 0.0), model="pinhole"); c.stop()                  # fed by hand here
    f = _render((640, 360), 0, 0.6, 0)
    assert not c.feed(f) and c.pattern == (9, 6) and "hold still" in c.hint                # first sight: not yet known to be still
    assert c.feed(f) and len(c.views) == 1                                                  # still: taken
    assert not c.feed(f) and "new spot" in c.hint                                           # the same view again: refused
    taken = 1
    for i, (p, a, s, t) in enumerate([((350, 200), 5, 0.4, 0), ((930, 200), -5, 0.4, 1), ((350, 520), 8, 0.45, -1), ((930, 520), -8, 0.45, 1), ((640, 360), 0, 0.9, 0)]):
        g = _render(p, a, s, t); c.feed(g); taken += c.feed(g)
    assert taken == 6 and c.cover.sum() >= 6
    assert not c.feed(np.full((H, W, 3), 128, np.uint8)) and "not found" in c.hint


def test_save_writes_the_calibration_and_archives_the_old_file(tmp_path, monkeypatch):
    p = tmp_path / "o4_camera.yaml"; shutil.copy(L.CONFIG, p); monkeypatch.setattr(L, "ARCHIVE", str(tmp_path / "arch"))
    before = p.read_text()
    arch = L.save({"model": "fisheye", "size": [W, H], "fx": 420.1, "fy": 421.2, "cx": 642.3, "cy": 357.4, "dist": [0.05, -0.02, 0.01, -0.003], "rms_px": 0.31, "views": 25}, str(p))
    import yaml
    cfg = yaml.safe_load(p.read_text()); cal = cfg["calibration"]
    assert cal["model"] == "fisheye" and cal["size"] == [W, H] and abs(cal["fx"] - 420.1) < 1e-6 and cal["dist"][3] == -0.003
    assert cfg["variant"] == "pro" and "# DJI O4 camera as mounted" in p.read_text()          # the rest and the comments kept
    assert open(arch).read() == before and os.path.dirname(arch) == str(tmp_path / "arch")
