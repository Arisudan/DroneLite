import importlib.util, os, sys
import numpy as np
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
spec = importlib.util.spec_from_file_location("calibrate_kf", os.path.join(ROOT, "tools_scripts", "calibrate_kf.py")); cal = importlib.util.module_from_spec(spec); spec.loader.exec_module(cal)


def test_robust_sigma_ignores_outliers():
    rng = np.random.default_rng(0); x = rng.normal(0, 0.07, 5000); x[:50] = 5.0
    assert abs(cal.robust_sigma(x) - 0.07) < 0.01


def _seg(sigma, n=800, speed_scale=1.0, seed=1):
    rng = np.random.default_rng(seed); t = np.arange(n) / 25.0; v = 0.3 * np.sin(t * 0.5)
    return {"t": t, "range_m": np.full(n, 0.6) + rng.normal(0, 0.01, n), "flow_fwd": speed_scale * v + rng.normal(0, sigma, n), "flow_right": rng.normal(0, sigma, n), "flow_q": np.full(n, 150.0),
            "pitch_deg": rng.normal(0, 0.05, n), "roll_deg": rng.normal(0, 0.05, n), "fc_vx": v, "fc_vy": np.zeros(n), "file": "synthetic.csv"}


def test_flow_noise_is_recovered_despite_real_motion():
    sn = cal.sensor_noise([_seg(0.08), _seg(0.08, seed=2)], 30)
    assert abs(sn["flow_sigma_ms"] - 0.08) < 0.015


def test_flow_scale_against_the_ekf_speed():
    fe = cal.flow_vs_ekf([_seg(0.03, speed_scale=1.2)])
    assert abs(fe["flow_scale"] - 1.2) < 0.08 and abs(fe["flow_lag_s"]) < 0.1


def test_replay_reports_a_consistency_number():
    r = cal.replay([_seg(0.08)], 0.0064, 0.0027, 0.044, 1.0, 1.0, 1.0, 30, 1.0)
    assert 0.1 < r["nis_flow"] < 5.0 and r["accepted"] > 100
