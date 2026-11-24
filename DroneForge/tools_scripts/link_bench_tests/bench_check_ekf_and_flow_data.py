#!/usr/bin/env python3
"""Is the FC's own Kalman filter (EKF, CRSF 0xD4) and the optical flow data coming? Read-only, nothing is armed. All distances in metres.
  bench_check_ekf_and_flow_data.py --adapter /dev/serial/by-id/usb-Silicon_Labs_CP2102...      (drone powered, link up; ~10 s)
Move the drone by hand during the test (slide it along the floor, lift it a little): the FC EKF position/velocity and the flow velocities must change.
PASS needs: 0xD4 at >= 5 Hz, valid, FC height follows the range sensor, flow data at >= 5 Hz with quality > 0."""
import argparse, math, statistics as S, sys, time
sys.path.insert(0, "/home/htic/Music/DroneForge/software/pc")
from nimbus_core.core import NimbusCore
from nimbus_core import topics as T

ap = argparse.ArgumentParser(); ap.add_argument("--adapter", required=True); ap.add_argument("--secs", type=float, default=10.0)
a = ap.parse_args()
got = []
c = NimbusCore(); c.bus.subscribe(T.TELEMETRY, lambda m: got.append((time.monotonic(), m[0], m[1])))
c.connect(a.adapter); t0 = time.monotonic()
try:
    while time.monotonic() - t0 < a.secs:
        c.tick(); time.sleep(0.02)
finally:
    c.disconnect()
o = [d for _, n, d in got if n == "optrange"]; k = [d for _, n, d in got if n == "kinematic"]; lk = [d for _, n, d in got if n == "link"]
rate = lambda l: len(l) / a.secs
ok = True
def verdict(good, text):
    global ok; ok &= bool(good); print(("PASS  " if good else "FAIL  ") + text)
print(f"link quality: {lk[-1]['up_lq'] if lk else 'no link frames'}")
verdict(len(o) / a.secs >= 5, f"range+flow (0xD3): {rate(o):.1f} Hz")
if o:
    rg = [d["distance_m"] for d in o]; fv = [d["velocity_forward"] for d in o]; fr = [d["velocity_right"] for d in o]; q = [d["flow_quality"] for d in o]
    print(f"      range height (m): min {min(rg):.3f}  max {max(rg):.3f}")
    print(f"      flow velocity forward (m/s): min {min(fv):.3f} max {max(fv):.3f} | right (m/s): min {min(fr):.3f} max {max(fr):.3f}")
    verdict(max(q) > 0, f"flow quality: max {max(q)} (0 = the floor has no texture, or the sensor sees nothing)")
verdict(len(k) / a.secs >= 5, f"FC EKF (0xD4): {rate(k):.1f} Hz")
if k:
    v = sum(1 for d in k if d["valid"]); verdict(v == len(k), f"FC EKF valid in {v} of {len(k)} frames")
    z = [d["z_m"] for d in k]; x = [d["x_m"] for d in k]; y = [d["y_m"] for d in k]
    print(f"      FC EKF height z (m): {min(z):.3f} .. {max(z):.3f} | x (m): {min(x):.3f} .. {max(x):.3f} | y (m): {min(y):.3f} .. {max(y):.3f}")
    print(f"      FC EKF velocity vx (m/s): {min(d['vx_mps'] for d in k):.3f} .. {max(d['vx_mps'] for d in k):.3f} | vy (m/s): {min(d['vy_mps'] for d in k):.3f} .. {max(d['vy_mps'] for d in k):.3f}")
    if o:
        verdict(abs(S.median(z) - S.median(rg)) < 0.10, f"FC height median {S.median(z):.3f} m vs range sensor median {S.median(rg):.3f} m")
print("RESULT:", "ALL PASS: the FC Kalman filter and the optical flow are flowing" if ok else "NOT OK, see the FAIL lines")
sys.exit(0 if ok else 1)
