#!/usr/bin/env python3
"""FC height hold, hand-lift check. PROPS MUST BE OFF. Arms the FC with the props removed, asks the FC to hold height once you lift the drone by hand,
and shows whether the FC's throttle responds the right way: RAISE the drone -> the motor outputs must FALL, LOWER it -> they must RISE. All distances in metres.
  bench_test_fc_height_hold_handlift.py --adapter <CP2102 by-id> --fc <FC USB by-id>
Steps: arms at zero throttle (keep the drone still on the floor 6 s), sets a low base throttle, requests the hold ("hold at the current height"), then waits for you to lift
the drone ~0.2 to 0.4 m above the floor and hold it steady. The FC engages only when it is more than 0.15 m above the floor and steady (it says ' HH' in the flight-mode text).
Then raise and lower it by about 0.15 m a few times. Enter or Ctrl-C = stop (disarm). Max 90 s."""
import argparse, select, statistics as S, sys, time
sys.path.insert(0, "/home/htic/Music/DroneForge/software/pc")
from nimbus_core.core import NimbusCore
from nimbus_core import topics as T
from nimbus_pc.hover.hil import FcReader
from nimbus_pc.sender import Controls, CRSF_MIN

ap = argparse.ArgumentParser(); ap.add_argument("--adapter", required=True); ap.add_argument("--fc", required=True); ap.add_argument("--base-thr", type=float, default=0.25)
a = ap.parse_args()
print("THIS ARMS THE DRONE AND SPINS THE MOTORS. All four PROPELLERS must be physically OFF the motors (look at the drone now).")
if input("Type exactly  PROPS REMOVED  to continue: ").strip() != "PROPS REMOVED" or input("Second check, type exactly  I LOOKED AT ALL FOUR MOTORS  : ").strip() != "I LOOKED AT ALL FOUR MOTORS":
    sys.exit("not confirmed, nothing done")
HOLD_NOW_TICKS = 384                                   # 1120 us: 'hold at the current height' (the FC decodes 1100..1149 us this way)
fc = FcReader(a.fc)
core = NimbusCore(); tel = {"mode": "", "z": None, "vz": None, "range": None}
def on_tel(m):
    n, d = m
    if n == "flight_mode": tel["mode"] = d.get("mode", "")
    elif n == "kinematic": tel["z"], tel["vz"] = d["z_m"], d["vz_mps"]
    elif n == "optrange": tel["range"] = d["distance_m"]
core.bus.subscribe(T.TELEMETRY, on_tel)
core.connect(a.adapter); core.set_source("external"); core.set_allow_arm(True)
log = []; t0 = time.monotonic(); phase = "wait"; t_arm = None; engaged = None; ground = None
def send(thr, arm, aux5):
    core.set_external_controls(Controls(arm=arm, throttle=thr, aux5=aux5))
try:
    while time.monotonic() - t0 < 90:
        core.tick(); time.sleep(0.02); now = time.monotonic() - t0
        if select.select([sys.stdin], [], [], 0)[0] and sys.stdin.readline() != "": print("operator stop"); break
        if phase == "wait":
            if now > 4 and tel["mode"]:
                phase = "arming"; t_arm = now; print("arming at zero throttle: keep the drone STILL on the floor (gyro calibration)")
            continue
        armed = bool(tel["mode"]) and not tel["mode"].endswith("*")
        if phase == "arming":
            send(0.0, True, CRSF_MIN)
            if armed and now - t_arm > 6:
                phase = "hold"; ground = tel["range"]; print(f"armed. LIFT the drone by hand to about 0.3 m above the floor and keep it steady (floor reads {ground} m)")
            elif now - t_arm > 14: print("FC did not arm"); break
            continue
        send(a.base_thr, True, HOLD_NOW_TICKS)
        m = fc.motors()
        if m and len(m) >= 4 and int(now * 2) != int((now - 0.02) * 2):
            avg = sum(m[:4]) / 4; hh = " HH" in tel["mode"]
            if hh and engaged is None: engaged = now; print("FC HEIGHT HOLD ENGAGED")
            log.append((now, tel["range"], avg, hh))
            print(f"{now:5.1f}s  range {tel['range'] if tel['range'] is not None else float('nan'):.3f} m  fc z {tel['z'] if tel['z'] is not None else float('nan'):.3f} m  motors avg {avg:6.0f}  mode '{tel['mode']}'", flush=True)
finally:
    send(0.0, False, CRSF_MIN); time.sleep(0.5); core.safe_now(); time.sleep(0.3); core.disconnect(); fc.close()
hh = [(r, m) for _, r, m, h in log if h and r is not None]
print("\n--- result ---")
if not hh: print("FAIL: the FC never engaged the hold (' HH' not seen). Was the drone lifted above 0.15 m and held steady?")
else:
    lo = [m for r, m in hh if r < S.median([x for x, _ in hh])]; hi = [m for r, m in hh if r >= S.median([x for x, _ in hh])]
    print(f"engaged for {len(hh)} samples; heights {min(r for r,_ in hh):.3f}..{max(r for r,_ in hh):.3f} m; motor average when LOWER {S.mean(lo):.0f}, when HIGHER {S.mean(hi):.0f}")
    print("PASS: the throttle is higher when the drone is lower (the hold pushes it back up)" if S.mean(lo) > S.mean(hi) + 3 else
          "NOT CLEAR: move the drone up and down more (about 0.15 m) while it is engaged, or the hold does not respond; send me this output")
