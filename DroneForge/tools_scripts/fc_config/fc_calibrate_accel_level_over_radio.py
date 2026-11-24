#!/usr/bin/env python3
"""Accelerometer LEVEL calibration WITHOUT USB: the FC runs on the battery exactly as in flight, the calibration stick gesture is sent over the radio.
THIS DOES NOT ARM THE DRONE and the motors do not spin: the arm channel stays LOW the whole time. It sends the Betaflight gesture THROTTLE FULL + YAW LEFT + PITCH BACK + ROLL CENTRE
(accelerometer calibration, accepted only while DISARMED) for 2.5 s, then centres the sticks. The FC measures the accelerometer for ~0.5 s and SAVES the result by itself.
Needs: drone resting where it will take off, completely still, battery connected, FC USB UNPLUGGED, link up.   usage: fc_calibrate_accel_level_over_radio.py --adapter <CP2102>"""
import argparse, math, statistics as S, sys, time
sys.path.insert(0, "/home/htic/Music/DroneForge/software/pc")
from nimbus_core.core import NimbusCore
from nimbus_core import topics as T
from nimbus_pc.sender import Controls

ap = argparse.ArgumentParser(); ap.add_argument("--adapter", required=True); a = ap.parse_args()
print("This does NOT arm the drone (arm switch stays low, motors do not spin). It makes the drone's CURRENT resting pose the 'level' reference.\n"
      "The drone must rest where it will take off, completely STILL, battery connected, FC USB unplugged.")
if input("Type exactly  LEVEL HERE  to continue: ").strip() != "LEVEL HERE":
    sys.exit("not confirmed, nothing done")
tel = {"mode": "", "att": []}
def on_tel(m):
    n, d = m
    if n == "flight_mode": tel["mode"] = d.get("mode", "")
    elif n == "attitude": tel["att"].append((time.monotonic(), math.degrees(d["pitch_rad"]), math.degrees(d["roll_rad"])))
core = NimbusCore(); core.bus.subscribe(T.TELEMETRY, on_tel)
core.connect(a.adapter); core.set_allow_arm(False); core.set_source("external")
def hold(c: Controls, secs: float):
    t = time.monotonic()
    while time.monotonic() - t < secs:
        core.set_external_controls(c); core.tick(); time.sleep(0.02)
def tilt(secs: float):
    t0 = time.monotonic(); hold(Controls(), secs)
    v = [(p, r) for t, p, r in tel["att"] if t >= t0]
    return (S.median(p for p, _ in v), S.median(r for _, r in v), max(p for p, _ in v) - min(p for p, _ in v), max(r for _, r in v) - min(r for _, r in v)) if v else None
try:
    hold(Controls(), 6.0)                                           # idle sticks, arm low: wait for telemetry
    if not tel["mode"] or not tel["mode"].endswith("*"):
        sys.exit(f"REFUSING: the FC does not report 'disarmed' (mode text {tel['mode']!r}); no telemetry, or it is armed")
    b = tilt(3.0)
    if not b: sys.exit("REFUSING: no attitude telemetry")
    print(f"BEFORE: pitch {b[0]:+.2f} deg  roll {b[1]:+.2f} deg   (noise span pitch {b[2]:.2f} roll {b[3]:.2f})")
    if b[2] > 0.4 or b[3] > 0.4: sys.exit("REFUSING: the drone is not still (attitude is moving): keep it completely still and retry")
    print("sending the calibration gesture for 2.5 s (arm stays LOW) ...")
    hold(Controls(roll=0.0, pitch=-1.0, yaw=-1.0, throttle=1.0, arm=False), 2.5)
    hold(Controls(), 4.0)                                           # sticks centred, throttle minimum: the FC finishes and saves
    af = tilt(3.0)
    print(f"AFTER : pitch {af[0]:+.2f} deg  roll {af[1]:+.2f} deg   (noise span pitch {af[2]:.2f} roll {af[3]:.2f})")
    print("RESULT:", "LEVEL (within 0.15 deg): saved by the FC itself. Do not move the drone; run the ramp or the GCS takeoff." if max(abs(af[0]), abs(af[1])) <= 0.15
          else "NOT within 0.15 deg: the gesture may not have been accepted (stick commands off? RX link?). Tell me the numbers.")
finally:
    core.safe_now(); time.sleep(0.4); core.disconnect()
