#!/usr/bin/env python3
"""FC SD card logs (Betaflight blackbox): copy them to the PC, then delete the OLDEST on the card until there is room for future logs.
Put the SD card in the reader, find its mount point (e.g. /media/htic/XXXX), then:
  fc_sd_copy_and_free_logs.py --card /media/htic/XXXX                  (copy everything to docs/fc_logs/<time>/, delete old logs until >= 50 % of the card is free)
  fc_sd_copy_and_free_logs.py --card /media/htic/XXXX --free-pct 80    (leave more free)
Files are deleted from the card ONLY after the copy is verified (same size). Empty (0 byte) logs are deleted without copying. Asks for 'yes' first."""
import argparse, os, shutil, sys, time
ap = argparse.ArgumentParser(); ap.add_argument("--card", required=True); ap.add_argument("--free-pct", type=float, default=50.0); ap.add_argument("--yes", action="store_true")
a = ap.parse_args()
if not os.path.isdir(a.card): sys.exit("not a folder: " + a.card)
logs = sorted((os.path.join(r, f) for r, _, fs in os.walk(a.card) for f in fs if f.upper().startswith("LOG") or f.lower().endswith((".bbl", ".bfl", ".txt"))), key=os.path.getmtime)
tot = shutil.disk_usage(a.card)
print(f"card: {tot.free / 1e6:.0f} MB free of {tot.total / 1e6:.0f} MB; {len(logs)} log files, {sum(os.path.getsize(p) for p in logs) / 1e6:.1f} MB")
for p in logs: print(f"  {os.path.getsize(p) / 1e3:10.1f} kB  {time.strftime('%Y-%m-%d %H:%M', time.localtime(os.path.getmtime(p)))}  {os.path.relpath(p, a.card)}")
if not logs: sys.exit("no logs on the card")
if not a.yes and input("copy them to the PC, then delete the oldest from the card until enough is free? type yes: ").strip().lower() != "yes": sys.exit("nothing done")
dest = f"/home/htic/Music/DroneForge/docs/fc_logs/{time.strftime('%Y%m%d_%H%M%S')}"; os.makedirs(dest, exist_ok=True)
saved = {}
for p in logs:
    if os.path.getsize(p) == 0: continue
    d = os.path.join(dest, os.path.relpath(p, a.card)); os.makedirs(os.path.dirname(d), exist_ok=True); shutil.copy2(p, d)
    saved[p] = os.path.getsize(d) == os.path.getsize(p)
print(f"copied {sum(saved.values())} of {len(saved)} non-empty logs to {dest}")
for p in logs:                                                         # oldest first
    free_pct = shutil.disk_usage(a.card).free / tot.total * 100
    empty = os.path.getsize(p) == 0
    if free_pct >= a.free_pct and not empty: continue
    if empty or saved.get(p):
        os.remove(p); print("deleted from card:", os.path.relpath(p, a.card))
    else: print("KEPT (copy not verified):", os.path.relpath(p, a.card))
print(f"card now {shutil.disk_usage(a.card).free / tot.total * 100:.0f} % free")
