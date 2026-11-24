#!/usr/bin/env python3
"""Range accuracy vs a ruler. DISARMED, props off. You hold a FLAT object (book/cardboard) under the lidar at each distance
(measure from the sensor window to the object with a ruler); a bell says when to hold and when you may move on.
Also logs telemetry gaps and link quality over the whole run (no link loss is expected).
usage: bench_test_lidar_range_accuracy.py <adapter_port> <out_file> [lead_s]"""
import collections, statistics as st, subprocess, sys, time
sys.path.insert(0, "/home/htic/Music/DroneForge/software/pc")
from nimbus_pc.sender import Controls
from nimbus_pc.session import Session

ADP, OUT = sys.argv[1], sys.argv[2]
LEAD = float(sys.argv[3]) if len(sys.argv) > 3 else 20.0
TARGETS_CM = [10, 20, 30, 50, 80, 100]
MOVE_S, HOLD_S = 8.0, 6.0
t0 = time.monotonic()
opt, lq = [], []          # (t, dist_m, quality, strength)  /  (t, lq)
arr = collections.defaultdict(list)
raw_d3, per_sec = [], collections.defaultdict(collections.Counter)


def on_frame(f):
    t = time.monotonic() - t0
    per_sec[int(t)][f.type] += 1
    if f.type == 0xD3: raw_d3.append((round(t, 3), f.payload.hex()))


def on_tel(n, d):
    t = time.monotonic() - t0
    arr[n].append(t)
    if n == "optrange": opt.append((t, d["distance_m"], d["flow_quality"], d["dist_strength"]))
    if n == "link": lq.append((t, d.get("uplink_lq", d.get("lq", None))))


def bell(msg):
    subprocess.Popen(["pw-play", "/usr/share/sounds/freedesktop/stereo/bell.oga"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.Popen(["notify-send", "-u", "critical", "-t", "6000", "Range test", msg], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print(f"[{time.monotonic()-t0:5.1f}s] {msg}", flush=True)


sched, t = [], LEAD
for cm in TARGETS_CM:
    sched.append((cm, t, t + MOVE_S, t + MOVE_S + HOLD_S)); t += MOVE_S + HOLD_S
END = t + 3
s = Session(ADP, hz=100.0, allow_arm=False, on_telemetry=on_tel, on_frame=on_frame)
s.start(); nxt = 0
try:
    while time.monotonic() - t0 < END:
        now = time.monotonic() - t0
        s.set_controls(Controls())
        if nxt < len(sched) and now >= sched[nxt][1]:
            bell(f"Hold the flat object {sched[nxt][0]} cm from the sensor, keep it still"); nxt += 1
        time.sleep(0.01)
finally:
    s.set_controls(Controls()); time.sleep(0.1); s.stop()
bell("Test finished")
L = ["target_cm | measured median cm | error cm | sd mm | samples | flow quality | signal strength"]
for cm, ts, hs, he in sched:
    w = [x for x in opt if hs <= x[0] < he]
    if len(w) < 5: L.append(f"{cm:5d} | no samples ({len(w)})"); continue
    d = [x[1] * 100 for x in w]; m = st.median(d)
    L.append(f"{cm:5d} | {m:6.1f} | {m-cm:+5.1f} | {st.pstdev(d)*10:5.1f} | {len(w):3d} | {st.mean(x[2] for x in w):5.0f} | {st.mean(x[3] for x in w):5.0f}")
L.append("")
for n in ("optrange", "attitude", "battery", "link"):
    t_ = [x for x in arr[n] if x >= 8.0]
    if len(t_) > 5:
        g = [b - a for a, b in zip(t_, t_[1:] + [END - 3])]   # include the silence at the END (a frozen TX hides there)
        L.append(f"{n}: {len(t_)/(t_[-1]-t_[0]):.1f} Hz, max gap {max(g)*1000:.0f} ms, gaps>0.3 s: {sum(x>0.3 for x in g)}")
q = [x[1] for x in lq if x[1] is not None]
if q: L.append(f"uplink LQ min {min(q)} mean {st.mean(q):.0f}")
open(OUT, "w").write("\n".join(L) + "\n"); print("\n".join(L))
with open(OUT + ".frames.txt", "w") as f:
    f.write("per second frame-type counts (hex type:count)\n")
    for sec in sorted(per_sec): f.write(f"{sec:4d} " + " ".join(f"{k:02x}:{v}" for k, v in sorted(per_sec[sec].items())) + "\n")
    f.write("\nall 0xD3 payloads (t, hex)\n")
    for t, h in raw_d3: f.write(f"{t} {h}\n")
