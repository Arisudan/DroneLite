#!/usr/bin/env python3
"""VO against the EKF from one hold-log CSV (2026-11-26). The EKF (after v4.8.1) is the reference: this tells you the SIGN and SCALE of the visual odometry per axis.

Use: run the GCS with VO (and DEPTH) on, drone DISARMED in your hand, a textured surface; wait for "key frame set"; then slide the drone slowly about 0.5 m forward, back, right, back
(any distance, the EKF measures it); stop; send / read the newest docs/status_logs/hold_live/*.csv:
    python3 tools_scripts/vo_vs_ekf.py <hold.csv>
For every VO sample ("from key frame: x fwd, y right" or the bench text) it pairs the EKF displacement integrated since the key frame was set, then fits VO = k * EKF per axis.
k near +1 = right sign and scale; k negative = VO inverted on that axis; |k| far from 1 = VO scale (depth) is off by that factor. Needs enough motion: it says so when it does not have it."""
from __future__ import annotations

import csv
import math
import re
import sys

import numpy as np

KF = re.compile(r"from key frame: ([+-]?\d+\.\d+) m fwd, ([+-]?\d+\.\d+) m right")
BENCH = re.compile(r"VO moved forward ([+-]?\d+\.\d+) m, right ([+-]?\d+\.\d+) m")


def pairs(path: str):
    rows = list(csv.DictReader(open(path)))
    px = py = 0.0; pt = None; out = []
    for r in rows:
        t = float(r["t_s"]); msg = (r.get("vo_msg") or "") + " " + (r.get("vo_text") or "")
        if r.get("fc_vfwd_mps"):
            if pt is not None and t - pt < 1.5: px += float(r["fc_vfwd_mps"]) * (t - pt); py += float(r["fc_vright_mps"]) * (t - pt)
            pt = t
        if "key frame set" in (r.get("vo_msg") or ""): px = py = 0.0
        m = KF.search(msg) or BENCH.search(msg)
        if m: out.append((t, float(m.group(1)), float(m.group(2)), px, py))
    return out


def fit(vo: np.ndarray, ekf: np.ndarray):
    if len(vo) < 6 or ekf.std() < 0.05 or float(ekf @ ekf) < 1e-9: return None
    k = float(vo @ ekf) / float(ekf @ ekf); c = float(np.corrcoef(vo, ekf)[0, 1]) if vo.std() > 1e-6 else float("nan")
    return k, c


def main(path: str) -> int:
    p = pairs(path)
    if not p: print("no VO samples in this CSV (was VO on, did it set a key frame?)"); return 1
    a = np.array(p)
    print(f"{len(p)} VO samples, t {a[0,0]:.0f}-{a[-1,0]:.0f} s")
    for i, name in ((1, "forward"), (2, "right")):
        ekf = a[:, i + 2]; vo = a[:, i]; f = fit(vo, ekf)
        print(f"  {name:8s}: VO range [{vo.min():+.2f},{vo.max():+.2f}] m, EKF range [{ekf.min():+.2f},{ekf.max():+.2f}] m", end="  ")
        if f is None: print("-> not enough motion on this axis to judge (move it at least 0.3 m)"); continue
        k, c = f
        verdict = "INVERTED" if k < -0.3 and c < -0.5 else ("SIGN OK" if k > 0.3 and c > 0.5 else "unclear")
        print(f"-> k = {k:+.2f}, corr {c:+.2f}: {verdict}" + (f"; scale off by x{abs(k):.1f}" if verdict == "SIGN OK" and not 0.7 < k < 1.4 else ""))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1]) if len(sys.argv) > 1 else print(__doc__) or 1)
