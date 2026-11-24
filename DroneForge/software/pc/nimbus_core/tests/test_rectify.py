import numpy as np
from nimbus_core.rectify import Rectifier, nominal_K, PROFILE


def test_nominal_K_matches_the_diagonal_fov():
    K = nominal_K(1280, 720, 117.6); import math
    assert abs(2 * math.degrees(math.atan(math.hypot(1280, 720) / 2 / K[0, 0])) - 117.6) < 1e-6


def test_straight_lines_stay_straight_and_the_profile_is_met():
    # a synthetic checkerboard-like picture at the stream size; an ideal lens must map to the pinhole profile as a pure crop+scale
    img = np.zeros((720, 1280, 3), np.uint8); img[::40, :] = 255; img[:, ::40] = 255
    out = Rectifier(1280, 720, diagonal_fov_deg=117.6)(img)
    assert out.shape == (PROFILE["height"], PROFILE["width"], 3) and out.max() == 255
    col = out[:, 320, 0]; rows = np.nonzero(col > 200)[0]                      # the vertical centre line of a horizontal grid line stays one line
    assert len(rows) > 3


def test_equidistant_lens_maps_angles_correctly():
    """O4 Pro nominal lens: a source pixel at angle t from the centre sits at radius f*t; the rectified pinhole pixel for the same ray sits at fx_new*tan(t)."""
    import math
    r = Rectifier(1280, 720, diagonal_fov_deg=155.0, lens="equidistant"); f = nominal_K(1280, 720, 155.0, "equidistant")[0, 0]
    assert abs(f - (math.hypot(1280, 720) / 2) / math.radians(77.5)) < 1e-6
    for th in (0.2, 0.5, 0.8):                                       # rays along +x at angle th: output pixel (320 + fx tan th, 240)
        u = 320 + PROFILE["fx"] * math.tan(th); mx = r.m1[240, int(round(u))]; src_x = float(mx[0]) if hasattr(mx, "__len__") else float(mx)
        assert abs((src_x - 640) - f * th) < 3.0                      # source radius = f * th (pixels)
