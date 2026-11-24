#!/usr/bin/env python3
"""Calibrate the PC-side Kalman filter (software/pc/nimbus_core/kalman.py) from REAL flight data of our drone. Reads only; arms and flies nothing.

Data used
  GCS flight recordings (docs/flights/*real-fc.csv, 25 Hz): range, optical flow (forward/right, quality), pitch/roll, the FC's own EKF velocity.
  FC Blackbox logs (docs/fc_logs/**/LOG*.BFL): gyro samples while the motors spin, for the gyro noise.
What it measures
  flow noise (robust sigma of the flow after removing its slow part), by quality bin; range noise vs height; attitude noise; gyro noise (decimated to the telemetry rate);
  flow scale and delay against the FC EKF speed; and a regression dv/dt = K*tilt - D*v on the real data (reported with its R^2, applied only if it is meaningful).
Then it replays the filter over every airborne segment and tunes the flow variance until the normalised innovation squared (NIS) averages 1 (a consistent filter),
and writes gazebo/config/kf_calib.json, which the simulated sensors (vision/sim_tf_odom.py) and the filter read.
Limits (stated in the report): the recordings have no yaw, so the replay assumes a constant heading; hover-like flights carry little information about drag or tilt gain.
usage: python3 tools_scripts/calibrate_kf.py [--flights GLOB] [--bfl GLOB] [--out PATH]"""
import argparse, csv, glob, json, math, os, sys
import numpy as np
from scipy.ndimage import uniform_filter1d
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "software", "pc"))
from nimbus_core.kalman import LateralKF, Plant, FlowParams


def robust_sigma(x):
    x = np.asarray(x, float); x = x[np.isfinite(x)]
    return float(1.4826 * np.median(np.abs(x - np.median(x)))) if len(x) > 20 else float("nan")


def hp(x, win):
    x = np.asarray(x, float); return x - uniform_filter1d(x, max(3, int(win)), mode="nearest")


def _f(v):
    try: return float(v) if v not in ("", None) else float("nan")
    except ValueError: return float("nan")


def load_segments(files, min_range=0.15, min_len_s=2.0, max_gap_s=0.3):
    """Contiguous airborne stretches (phase flying, range above min_range) from the GCS recordings."""
    segs = []
    for f in files:
        try: rows = list(csv.DictReader(open(f, newline="")))
        except Exception: continue
        cur = []
        def flush():
            if len(cur) >= min_len_s * 25:
                a = {k: np.array([_f(r[k]) for r in cur]) for k in ("t", "range_m", "flow_fwd", "flow_right", "flow_q", "pitch_deg", "roll_deg", "fc_vx", "fc_vy")}
                a["file"] = os.path.basename(f); segs.append(a)
        for r in rows:
            ok = r.get("phase") == "flying" and _f(r.get("range_m")) > min_range and not math.isnan(_f(r.get("flow_fwd")))
            if ok and (not cur or _f(r["t"]) - _f(cur[-1]["t"]) < max_gap_s): cur.append(r)
            else: flush(); cur = [r] if ok else []
        flush()
    return segs


def sensor_noise(segs, min_q):
    cat = lambda k: np.concatenate([s[k] for s in segs])
    res_f, res_r, qs = [], [], []
    for s in segs:
        good = s["flow_q"] >= min_q
        res_f.append(hp(s["flow_fwd"], 25)[good]); res_r.append(hp(s["flow_right"], 25)[good]); qs.append(s["flow_q"][good])
    rf, rr, q = np.concatenate(res_f), np.concatenate(res_r), np.concatenate(qs); both = np.concatenate([rf, rr])
    out = {"flow_sigma_ms": robust_sigma(both), "flow_samples": int(len(both))}
    for name, lo, hi in (("low (q<100)", min_q, 100), ("high (q>=100)", 100, 256)):
        m = (q >= lo) & (q < hi); out["flow_sigma_" + name.split()[0]] = robust_sigma(np.concatenate([rf[m], rr[m]])) if m.sum() > 50 else float("nan")
    # range noise vs height
    h = np.concatenate([uniform_filter1d(s["range_m"], 13, mode="nearest") for s in segs]); rr_ = np.concatenate([hp(s["range_m"], 13) for s in segs]); xs, ys = [], []
    for lo, hi in ((0.15, 0.4), (0.4, 0.8), (0.8, 1.5), (1.5, 4.0)):
        m = (h >= lo) & (h < hi)
        if m.sum() > 100: xs.append(float(np.mean(h[m]))); ys.append(robust_sigma(rr_[m]))
    if len(xs) >= 2: b, a = np.polyfit(xs, ys, 1); out["range_sigma_a"], out["range_sigma_b"] = float(max(a, 0.002)), float(max(b, 0.0))
    else: out["range_sigma_a"], out["range_sigma_b"] = (float(ys[0]) if ys else 0.02), 0.0
    out["range_bins"] = list(zip([round(x, 2) for x in xs], [round(y, 4) for y in ys]))
    # attitude noise (deg -> rad)
    att = np.concatenate([np.concatenate([hp(s["pitch_deg"], 13), hp(s["roll_deg"], 13)]) for s in segs]); out["att_sigma_rad"] = math.radians(robust_sigma(att))
    return out


def flow_vs_ekf(segs):
    """Scale and delay of the flow speed against the FC EKF speed (no yaw needed: speeds)."""
    best = None; num = den = 0.0; n = 0; lag_scores = {}
    for lag in range(-15, 16):                                           # +-0.6 s at 25 Hz
        sx = sy = 0.0; c = 0
        for s in segs:
            sf = np.hypot(s["flow_fwd"], s["flow_right"]); se = np.hypot(s["fc_vx"], s["fc_vy"]); ok = np.isfinite(sf) & np.isfinite(se)
            a, b = (sf[lag:], se[:len(se) - lag]) if lag >= 0 else (sf[:lag], se[-lag:])
            ok = np.isfinite(a) & np.isfinite(b) & (b > 0.1)
            sx += float(np.sum(a[ok] * b[ok])); sy += float(np.sum(b[ok] ** 2)); c += int(ok.sum())
        if c > 30: lag_scores[lag] = (sx / sy if sy else float("nan"), c, sx)
    if not lag_scores: return {"flow_scale": 1.0, "flow_lag_s": 0.0, "flow_vs_ekf_samples": 0}
    lag = max(lag_scores, key=lambda k: lag_scores[k][2] / max(1.0, math.sqrt(lag_scores[k][1])))
    return {"flow_scale": float(lag_scores[lag][0]), "flow_lag_s": lag / 25.0, "flow_vs_ekf_samples": int(lag_scores[lag][1])}


def plant_regression(segs, sign_p=None):
    """dv/dt = K*tilt - D*v from the real data; returns (K, D, R2, n, pitch_sign). Meaningful only with enough tilt variation."""
    rows = []
    for s in segs:
        dt = np.diff(s["t"]); ok = (dt > 0.02) & (dt < 0.1)
        for v, th in ((s["flow_fwd"], np.radians(s["pitch_deg"])), (s["flow_right"], np.radians(s["roll_deg"]))):
            vs = uniform_filter1d(v, 5, mode="nearest"); acc = np.diff(vs) / np.where(dt > 0, dt, np.nan); X = np.column_stack([uniform_filter1d(th, 5, mode="nearest")[:-1], -vs[:-1]])
            m = ok & np.isfinite(acc) & np.all(np.isfinite(X), 1) & (np.abs(acc) < 6); rows.append((X[m], acc[m]))
    X = np.vstack([r[0] for r in rows]); y = np.concatenate([r[1] for r in rows])
    if len(y) < 200: return None
    coef, *_ = np.linalg.lstsq(X, y, rcond=None); pred = X @ coef; r2 = 1 - float(np.sum((y - pred) ** 2) / max(np.sum((y - y.mean()) ** 2), 1e-9))
    return {"K_signed": float(coef[0]), "D": float(coef[1]), "r2": r2, "n": int(len(y)), "tilt_std_deg": float(np.degrees(np.std(X[:, 0])))}


def gyro_noise(bfl_files, rate_hz=50.0, max_files=6):
    sys.path.insert(0, os.path.join(ROOT, "software", "pc"))
    from nimbus_gcs import bfl
    sig, used = [], []
    for f in sorted(bfl_files, key=os.path.getsize, reverse=True)[:max_files]:
        try: lg = bfl.Log(f); lg.decode()
        except Exception: continue
        t = np.array(lg.column("time"), float) / 1e6
        if len(t) < 1000: continue
        fs = len(t) / (t[-1] - t[0]); mot = np.mean([np.array(lg.column(f"motor[{i}]"), float) for i in range(4)], axis=0); spin = mot > 1100; stride = max(1, int(round(fs / rate_hz)))
        for ax in (0, 1):
            g = np.array(lg.column(f"gyroADC[{ax}]"), float)[spin][::stride]
            if len(g) < 200: continue
            d2 = g[2:] - 2 * g[1:-1] + g[:-2]; sig.append(robust_sigma(d2) / math.sqrt(6.0))        # second difference: immune to slow real motion
        used.append(os.path.basename(f))
    return {"gyro_sigma_rad_s": math.radians(float(np.nanmedian(sig))) if sig else float("nan"), "gyro_files": used}


def replay(segs, r0, att_var, gyro_var, flow_scale, pitch_sign, roll_sign, min_q, q_scale=1.0):
    nis_f, nis_a, rej, acc, errs = [], [], 0, 0, []
    for s in segs:
        kf = LateralKF(Plant(), FlowParams(r0=r0, min_quality=min_q), att_var, gyro_var, q_scale); kf.log_nis = True; kf.flying = True
        for i in range(len(s["t"])):
            t = s["t"][i]
            if not np.isfinite(t): continue
            kf.predict(t, 0.0, True)
            if np.isfinite(s["pitch_deg"][i]) and np.isfinite(s["roll_deg"][i]): kf.update_attitude(roll_sign * math.radians(s["roll_deg"][i]), pitch_sign * math.radians(s["pitch_deg"][i]))
            if np.isfinite(s["flow_fwd"][i]): kf.update_flow(s["flow_fwd"][i] / flow_scale, s["flow_right"][i] / flow_scale, max(s["range_m"][i], 0.05), int(s["flow_q"][i]) if np.isfinite(s["flow_q"][i]) else 0)
            if np.isfinite(s["fc_vx"][i]): errs.append(abs(math.hypot(*kf.vel_world) - math.hypot(s["fc_vx"][i], s["fc_vy"][i])))
        nis_f += kf.nis.get("flow", []); nis_a += kf.nis.get("attitude", []); rej += kf.rejected; acc += kf.accepted
    return {"nis_flow": float(np.mean(nis_f)) if nis_f else float("nan"), "nis_att": float(np.mean(nis_a)) if nis_a else float("nan"), "rejected": rej, "accepted": acc,
            "speed_rms_vs_fc_ekf": float(np.sqrt(np.mean(np.square(errs)))) if errs else float("nan")}


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--flights", default=os.path.join(ROOT, "docs", "flights", "*real-fc.csv")); ap.add_argument("--bfl", default=os.path.join(ROOT, "docs", "fc_logs", "**", "*.BFL"))
    ap.add_argument("--out", default=os.path.join(ROOT, "gazebo", "config", "kf_calib.json")); ap.add_argument("--min-quality", type=int, default=30); a = ap.parse_args()
    segs = load_segments(sorted(glob.glob(a.flights)))
    if not segs: print("no airborne segments found in", a.flights); sys.exit(1)
    air = sum(len(s["t"]) for s in segs) / 25.0; print(f"airborne data: {len(segs)} segments from {len(set(s['file'] for s in segs))} flights, {air:.0f} s")
    sn = sensor_noise(segs, a.min_quality); fe = flow_vs_ekf(segs); pr = plant_regression(segs); gy = gyro_noise(sorted(glob.glob(a.bfl, recursive=True)))
    print(f"flow noise {sn['flow_sigma_ms']:.3f} m/s (low quality {sn['flow_sigma_low']:.3f}, high quality {sn['flow_sigma_high']:.3f}), {sn['flow_samples']} samples")
    print(f"range noise {sn['range_sigma_a']:.3f} + {sn['range_sigma_b']:.3f}*h m   bins {sn['range_bins']}")
    print(f"attitude noise {math.degrees(sn['att_sigma_rad']):.2f} deg   gyro noise {math.degrees(gy['gyro_sigma_rad_s']) if gy['gyro_sigma_rad_s']==gy['gyro_sigma_rad_s'] else float('nan'):.2f} deg/s ({len(gy['gyro_files'])} Blackbox logs)")
    print(f"flow vs FC EKF speed: scale {fe['flow_scale']:.3f}, flow delay {fe['flow_lag_s']*1000:.0f} ms ({fe['flow_vs_ekf_samples']} samples)")
    if pr: print(f"real-data regression dv/dt = K*tilt - D*v: K {pr['K_signed']:+.1f}, D {pr['D']:.2f}, R2 {pr['r2']:.2f}, tilt std {pr['tilt_std_deg']:.2f} deg ({pr['n']} samples)")
    ps = -1.0 if (pr and pr["K_signed"] < 0) else 1.0                  # Betaflight attitude signs: choose the sign that makes tilt push the speed the right way
    r0 = max(sn["flow_sigma_ms"], 0.01) ** 2; att_var = max(sn["att_sigma_rad"], 0.002) ** 2; gyro_var = max(gy["gyro_sigma_rad_s"], 0.005) ** 2 if gy["gyro_sigma_rad_s"] == gy["gyro_sigma_rad_s"] else 0.043865
    q_scale = 1.0                                                      # the flow variance stays at the MEASURED sensor noise; the process noise is what is tuned until the filter is consistent (NIS ~ 1)
    for it in range(8):
        rp = replay(segs, r0, att_var, gyro_var, 1.0, ps, ps, a.min_quality, q_scale)
        print(f"  iteration {it}: process-noise scale {q_scale:.4f}  NIS(flow) {rp['nis_flow']:.2f}  NIS(att) {rp['nis_att']:.2f}  rejected {rp['rejected']}/{rp['rejected']+rp['accepted']}  speed rms vs FC EKF {rp['speed_rms_vs_fc_ekf']:.3f} m/s")
        if not (rp["nis_flow"] == rp["nis_flow"]) or abs(rp["nis_flow"] - 1.0) < 0.08: break
        q_scale = max(1e-4, q_scale * float(np.clip(rp["nis_flow"], 0.2, 3.0)))
    notes = ["the flow-vs-FC-EKF scale is only reported (which of the two reads right is not known without an independent speed), it is not applied", f"{air:.0f} s of real airborne data, mostly hover-like: drag and tilt gain are NOT identified from it (use the sysid in the simulator, or a real tilt-step flight)", "recordings have no yaw: replay assumes a constant heading"]
    plant_applied = bool(pr and pr["r2"] > 0.3 and abs(pr["K_signed"]) > 3)
    out = {"q_scale": q_scale, "flow_r0": r0, "flow_sigma_ms": math.sqrt(r0), "flow_scale_vs_fc_ekf": fe["flow_scale"], "flow_lag_s": fe["flow_lag_s"], "range_sigma_a": sn["range_sigma_a"], "range_sigma_b": sn["range_sigma_b"],
           "att_sigma_rad": math.sqrt(att_var), "att_var": att_var, "gyro_sigma_rad_s": math.sqrt(gyro_var), "gyro_var": gyro_var, "nis_flow_final": rp["nis_flow"], "airborne_s": air,
           "flights": len(set(s["file"] for s in segs)), "gyro_files": gy["gyro_files"], "real_plant_regression": pr, "real_plant_applied": plant_applied, "notes": notes}
    if plant_applied: out["k_real"], out["d_real"] = abs(pr["K_signed"]), max(pr["D"], 0.0)
    os.makedirs(os.path.dirname(a.out), exist_ok=True); json.dump(out, open(a.out, "w"), indent=1); print("written", a.out)
    for n in notes: print("note:", n)


if __name__ == "__main__": main()
