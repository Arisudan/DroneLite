#!/usr/bin/env python3
"""Cue, wait, then report stats of recorder CSV for a time window.
usage: mtf02p_report_window_stats.py <csv> <label> <ruler_cm> [grace_s=2] [window_s=4]
"""
import csv, statistics as st, subprocess, sys, time
path, label, ruler = sys.argv[1], sys.argv[2], float(sys.argv[3])
grace = float(sys.argv[4]) if len(sys.argv) > 4 else 2.0
win = float(sys.argv[5]) if len(sys.argv) > 5 else 4.0
subprocess.Popen(["pw-play", "/usr/share/sounds/freedesktop/stereo/bell.oga"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
subprocess.Popen(["spd-say", "-w", f"Sampling {label}. Hold steady."], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
t_start = time.time() + grace
time.sleep(grace + win + 0.8)
rows = [r for r in csv.DictReader(open(path)) if t_start <= float(r["t"]) <= t_start + win]
if not rows:
    print("NO FRAMES in window: recorder not running or FC not in passthrough"); sys.exit(1)
d = [int(r["dist_mm"]) for r in rows]; q = [int(r["flow_q"]) for r in rows]
med = int(st.median(d)); exp = ruler * 10
print(f"{label}: n={len(d)} ruler={exp:.0f} mm | median={med} min={min(d)} max={max(d)} sd={st.pstdev(d):.1f} mm | flow_q={int(st.median(q))} strength={int(st.median(int(r['strength']) for r in rows))}")
print(f"  error vs ruler: {med-exp:+.0f} mm ({100*(med-exp)/exp:+.0f}%)")
subprocess.Popen(["pw-play", "/usr/share/sounds/freedesktop/stereo/complete.oga"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
