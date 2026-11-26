#!/usr/bin/env python3
"""EKF bench sign test, no GCS window (2026-11-26). DISARMED ONLY: it never arms, the motors stay off.

What it does: connects to the radio dongle like the GCS (core + 100 Hz sender), prints the FC's own velocity estimate live
(forward / right in m/s), and after every move it prints the distance moved (the velocity integrated until the drone is still again).
The hold log CSV is written as in the GCS (docs/status_logs/hold_live/<date>_<time>_hold.csv, 4 Hz while disarmed).

Run (close the GCS first: only one program can hold the radio port):
    cd ~/DroneLite/DroneForge        (or ~/Music/DroneForge)
    python3 tools_scripts/ekf_bench_live.py                       # port = the CP2102 in /dev/serial/by-id
    python3 tools_scripts/ekf_bench_live.py --port /dev/serial/by-id/usb-Silicon_Labs_CP2102...-port0
    python3 tools_scripts/ekf_bench_live.py --sim                 # self-test without hardware
Then hold the drone level ~20 cm over a textured surface, nose at a fixed spot, and slide it slowly (3 s):
    forward 0.5 m, pause, back, pause, right 0.5 m, pause, back.   Expected: forward move prints "forward +", right move prints "right +".
Ctrl+C stops it. The VO check needs the GCS (video): use the GCS bench test for that part."""
from __future__ import annotations

import argparse
import glob
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "software", "pc"))
from nimbus_core.core import NimbusCore          # noqa: E402

STILL_MPS, STILL_S, MIN_MOVE_M = 0.03, 1.5, 0.15


def word(v: float, pos: str, neg: str) -> str:
    return f"{pos:>7s}" if v > 0.05 else (f"{neg:>7s}" if v < -0.05 else f"{'-':>7s}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--port"); ap.add_argument("--sim", action="store_true", help="self-test on the simulator")
    ap.add_argument("--seconds", type=float, default=0.0, help="stop after this long (0 = until Ctrl+C)")
    a = ap.parse_args()
    core = NimbusCore()
    if a.sim:
        core.connect_sim()
    else:
        port = a.port or next(iter(sorted(glob.glob("/dev/serial/by-id/*CP2102*"))), None)
        if not port: print("no radio dongle found in /dev/serial/by-id (is the GCS still running? close it first)"); return 1
        core.connect(port)
    print("connected. DISARMED ONLY. Move the drone by hand; Ctrl+C to stop.\n")
    print("   t    range  fwd m/s  right m/s   direction              | move since still: fwd m   right m")
    t0 = time.monotonic(); next_print = 0.0; last_t = t0; px = py = 0.0; still_since = None; moving = False
    try:
        while a.seconds <= 0 or time.monotonic() - t0 < a.seconds:
            core.tick(); time.sleep(0.02)
            now = time.monotonic(); L = core.hold_log.last
            kin = L.get("kinematic") or {}; rg = L.get("optrange") or {}
            vf, vr = kin.get("vx_mps"), kin.get("vy_mps")
            if vf is None or vr is None: continue
            dt = now - last_t; last_t = now; px += vf * dt; py += vr * dt
            sp = math.hypot(vf, vr)
            if sp < STILL_MPS:
                still_since = still_since or now
                if moving and now - still_since >= STILL_S:
                    if math.hypot(px, py) >= MIN_MOVE_M:
                        print(f"   >>> MOVE DONE: forward {px:+.2f} m, right {py:+.2f} m   (EKF: {'forward' if px > 0.1 else 'BACK' if px < -0.1 else '-'} / {'right' if py > 0.1 else 'LEFT' if py < -0.1 else '-'})")
                    px = py = 0.0; moving = False
            else:
                still_since = None; moving = True
            if now - t0 >= next_print:
                next_print = now - t0 + 0.5
                print(f"{now - t0:6.1f}  {rg.get('distance_m', float('nan')):5.2f}  {vf:+7.2f}  {vr:+9.2f}   {word(vf, 'FORWARD', 'BACK')} {word(vr, 'RIGHT', 'LEFT')}   | {px:+9.2f} {py:+10.2f}")
    except KeyboardInterrupt:
        pass
    finally:
        core.hold_log.close(); core.disconnect()
        if core.hold_log.path: print(f"\nhold log: {core.hold_log.path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
