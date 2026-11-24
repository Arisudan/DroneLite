#!/usr/bin/env python3
"""Optical-flow sign/axis check. DISARMED, props off. Hold the drone LEVEL, nose pointing AWAY from you, 30-50 cm above a textured floor (rug, wood, paper with print; not a plain white table).
Bells: slide it 20-30 cm in the named direction over ~2 s (smooth), then the opposite way when told. Repeated 4x for forward/back and right/left.
Expected (drone frame): FORWARD -> velocity_forward > 0; RIGHT -> velocity_right > 0.
usage: bench_test_optical_flow_sign.py <adapter_port> <out_file> [lead_s]"""
import collections, statistics as st, subprocess, sys, time
sys.path.insert(0, "/home/htic/Music/DroneForge/software/pc")
from nimbus_pc.sender import Controls
from nimbus_pc.session import Session

ADP, OUT = sys.argv[1], sys.argv[2]
LEAD = float(sys.argv[3]) if len(sys.argv) > 3 else 20.0
MOVE, REST = 2.0, 1.5
REPS = {"forward": 4, "right": 2}
PLAN = []
t = LEAD
for axis, pos, neg in (("forward", "toward the NOSE (away from you)", "toward YOU (backwards)"), ("right", "toward your RIGHT hand", "toward your LEFT hand")):
    for _ in range(REPS[axis]):
        PLAN.append((axis, +1, pos, t)); t += MOVE + REST
        PLAN.append((axis, -1, neg, t)); t += MOVE + REST
    t += 4.0
END = t
t0 = time.monotonic(); rows = []; att = []


def on_tel(n, d):
    now = time.monotonic() - t0
    if n == "optrange": rows.append((now, d["velocity_forward"], d["velocity_right"], d["flow_quality"], d["distance_m"], d["flow_x_raw"], d["flow_y_raw"]))
    if n == "attitude": att.append((now, d["pitch_rad"], d["roll_rad"]))


def bell(msg):
    subprocess.Popen(["pw-play", "/usr/share/sounds/freedesktop/stereo/bell.oga"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.Popen(["notify-send", "-u", "critical", "-t", "2500", "Flow test", msg], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print(f"[{time.monotonic()-t0:5.1f}s] {msg}", flush=True)


s = Session(ADP, hz=100.0, allow_arm=False, on_telemetry=on_tel)
s.start(); nxt = 0
try:
    while time.monotonic() - t0 < END:
        now = time.monotonic() - t0
        s.set_controls(Controls())
        if nxt < len(PLAN) and now >= PLAN[nxt][3]:
            bell(f"slide {PLAN[nxt][2]}"); nxt += 1
        time.sleep(0.01)
finally:
    s.set_controls(Controls()); time.sleep(0.1); s.stop()
bell("flow test finished")
L = []
res = {}
for axis, sgn, name, ts in PLAN:
    w = [r for r in rows if ts + 0.3 <= r[0] <= ts + MOVE + 0.3 and r[3] >= 20]
    if len(w) < 5: L.append(f"{name[:24]:24s} t={ts:5.1f}: too few samples ({len(w)})"); continue
    vf = [r[1] for r in w]; vr = [r[2] for r in w]; q = st.mean(r[3] for r in w)
    pk = lambda v: max(v, key=abs)
    L.append(f"{name[:24]:24s} t={ts:5.1f}: peak fwd {pk(vf):+.2f} right {pk(vr):+.2f} m/s  mean fwd {st.mean(vf):+.2f} right {st.mean(vr):+.2f}  quality {q:.0f}  dist {st.mean(r[4] for r in w)*100:.0f} cm")
    res.setdefault(axis, []).append(sgn * (st.mean(vf) if axis == "forward" else st.mean(vr)))
    res.setdefault(axis + "_cross", []).append(sgn * (st.mean(vr) if axis == "forward" else st.mean(vf)))
L.append("")
for axis in ("forward", "right"):
    v = res.get(axis, [])
    if v:
        agree = sum(1 for x in v if x > 0.03); wrong = sum(1 for x in v if x < -0.03)
        L.append(f"{axis.upper()}: mean signed response {st.mean(v):+.3f} m/s over {len(v)} moves; right sign {agree}, wrong sign {wrong}; cross-axis leak {st.mean(res[axis + '_cross']):+.3f}")
        L.append(f"   -> " + ("SIGN OK" if agree >= 0.75 * len(v) and wrong == 0 else "SIGN WRONG (flip opticalflow_align / controller sign)" if wrong >= 0.75 * len(v) else "INCONCLUSIVE (move more smoothly / better floor texture)"))
q = [r[3] for r in rows]
L.append(f"overall flow quality mean {st.mean(q):.0f}, samples {len(rows)}")
open(OUT, "w").write("\n".join(L) + "\n")
open(OUT + ".csv", "w").write("t,vfwd,vright,quality,dist_m,flow_x_raw,flow_y_raw\n" + "\n".join(",".join(f"{x:.3f}" for x in r) for r in rows))
print("\n".join(L))
