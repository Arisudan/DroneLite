#!/usr/bin/env python3
"""Show what was logged in a flight (all distances in metres). No argument = the latest flight; or give a log path; --list shows all flights.
Logs are written automatically at every connection to docs/flights/flight_<time>.csv (+ .events.txt, .summary.txt)."""
import glob, os, sys
sys.path.insert(0, "/home/htic/Music/DroneForge/software/pc")
from nimbus_core.recorder import DIR, summarize
logs = sorted(glob.glob(os.path.join(DIR, "flight_*.csv")))
if "--list" in sys.argv:
    for p in logs[-40:]: print(os.path.basename(p), f"{os.path.getsize(p) / 1e3:.0f} kB")
    sys.exit()
args = [a for a in sys.argv[1:] if not a.startswith("--")]
p = args[0] if args else (logs[-1] if logs else None)
print(summarize(p) if p else "no flight logs yet")
