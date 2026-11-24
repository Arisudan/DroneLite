import math
import numpy as np
from nimbus_core.lateral_lqr import LateralLqr, PlantID, LqrConfig, deg_to_stick, stick_to_deg


def test_stick_curve_roundtrip_and_nimbus_points():
    for d in (-12.0, -3.0, 0.0, 0.5, 8.0, 30.0): assert abs(stick_to_deg(deg_to_stick(d)) - d) < 1e-6
    assert abs(stick_to_deg(1.0) - 60.0) < 1e-9                      # full stick = 60 deg in the Nimbus mapping
    assert 0.32 < deg_to_stick(8.0) < 0.34


def _fly(delay=0.15, noise=0.0, seconds=25.0, seed=1, target=(3.0, 0.0), wn=1.4):
    """Point-mass plant + 0.08 s attitude lag + dead time, controlled by LateralLqr toward a position reference."""
    pl = PlantID(); lq = LateralLqr(pl, LqrConfig(wn=wn)); rng = np.random.default_rng(seed); dt = 0.02
    x = np.zeros(2); v = np.zeros(2); tilt = np.zeros(2); buf = [np.zeros(2)] * int(delay / dt); path = []
    for i in range(int(seconds / dt)):
        meas_p = x + rng.normal(0, noise, 2); meas_v = v + rng.normal(0, noise, 2)
        u = np.array(lq.step(dt, 0.0, meas_p, target, meas_v, (0.0, 0.0), (0.0, 0.0)))            # yaw 0: forward = +x, right = -y
        buf.append(np.array([u[0], u[1]])); cmd = buf.pop(0); tilt += (cmd - tilt) * dt / 0.08
        af = pl.k_f * math.radians(tilt[0]) - pl.d_f * v[0]; ar = pl.k_r * math.radians(tilt[1]) - pl.d_f * (-v[1])
        a = np.array([af, -ar]); v += a * dt; x += v * dt; path.append(x.copy())
    return np.array(path), v


def test_reaches_the_position_reference_without_overshoot_blowup():
    path, v = _fly()
    assert abs(path[-1][0] - 3.0) < 0.1 and abs(path[-1][1]) < 0.1 and np.linalg.norm(v) < 0.05
    assert path[:, 0].max() < 3.0 * 1.25                                 # overshoot below 25 %


def test_stable_with_measurement_noise_and_longer_delay():
    path, v = _fly(delay=0.3, noise=0.03)
    assert abs(path[-250:, 0].mean() - 3.0) < 0.15 and np.abs(path[-250:, 0] - 3.0).max() < 0.5
