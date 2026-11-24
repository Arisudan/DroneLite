#!/usr/bin/env python3
"""How often does each telemetry frame reach the PC, and how old is it when used? Reads a GCS recording (docs/flights/*real-fc.csv).
usage: python3 tools_scripts/telemetry_rates.py [file.csv ...]   (default: the newest real-fc recording)
Rates are counted from the *_age_ms columns (an age drop = a new frame arrived). Use the flight part only: a long idle tail skews the averages."""
import csv, glob, os, sys
COLS = [("attitude", "age_att_ms"), ("D3 range/flow", "age_d3_ms"), ("D4 EKF state", "age_d4_ms"), ("battery", "age_batt_ms")]
def num(r, k):
    try: return float(r[k])
    except (TypeError, ValueError): return None
files = sys.argv[1:] or sorted(glob.glob(os.path.expanduser("~/Music/DroneForge/docs/flights/*real-fc.csv")))[-1:]
for fn in files:
    rows = list(csv.DictReader(open(fn)))
    fl = [r for r in rows if r.get("phase") in ("flying", "armed")]
    rows = fl or rows
    T = [num(r, "t") for r in rows]; span = sum(b - a for a, b in zip(T, T[1:]) if 0 < b - a < 1.0)   # only contiguous time, not the gaps between flights
    print(f"== {os.path.basename(fn)}  {span:.0f} s of armed/flying data")
    for name, c in COLS:
        a = [num(r, c) for r in rows]; n = 0; prev = None
        for v in a:
            if v is None: continue
            if prev is not None and v < prev - 1: n += 1
            prev = v
        va = sorted(x for x in a if x is not None)
        if va: print(f"  {name:14s} {n / span:5.1f} Hz | age median {va[len(va) // 2]:5.0f} ms, 95th {va[int(len(va) * .95)]:5.0f} ms")
