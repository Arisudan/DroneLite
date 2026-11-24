#!/usr/bin/env python3
"""Does the TX survive distances whose low byte equals a CRSF address? (the ELRS 4.x router bug; FC now prefixes 00 C8)
Risky readings (FC sends cm*10 mm): 490 mm = 0xEA (radio), 750 mm = 0xEE (TX), 1260 mm = 0xEC (RX).
DISARMED, props off. You slowly sweep a flat object under the lidar (about 2 cm/s) so every cm is sampled; bells mark each sweep.
Reports: samples at each risky distance, and whether the TX ever stopped forwarding (checked per second, INCLUDING the end of the run).
usage: bench_test_lidar_range_sweep_crsf_address.py <adapter_port> <out_file> [lead_s]"""
import collections, statistics as st, subprocess, sys, time
sys.path.insert(0, "/home/htic/Music/DroneForge/software/pc")
from nimbus_pc.sender import Controls
from nimbus_pc.session import Session

ADP, OUT = sys.argv[1], sys.argv[2]
LEAD = float(sys.argv[3]) if len(sys.argv) > 3 else 25.0
SWEEP_S, GAP_S = 60.0, 5.0
PLAN = [("UP", "from 20 cm to about 140 cm"), ("DOWN", "from 140 cm back to 20 cm"), ("UP", "from 20 cm to about 140 cm"), ("DOWN", "from 140 cm back to 20 cm")]
RISKY = {490: "0xEA radio", 750: "0xEE TX", 1260: "0xEC RX"}
t0 = time.monotonic()
per_sec = collections.defaultdict(collections.Counter)
d3 = []


def on_frame(f):
    t = time.monotonic() - t0
    per_sec[int(t)][f.type] += 1
    if f.type == 0xD3: d3.append((t, f.payload))


def bell(msg):
    subprocess.Popen(["pw-play", "/usr/share/sounds/freedesktop/stereo/bell.oga"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.Popen(["notify-send", "-u", "critical", "-t", "6000", "Sweep test", msg], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print(f"[{time.monotonic()-t0:5.1f}s] {msg}", flush=True)


starts = [LEAD + i * (SWEEP_S + GAP_S) for i in range(len(PLAN))]
END = starts[-1] + SWEEP_S + 4
s = Session(ADP, hz=100.0, allow_arm=False, on_frame=on_frame)
s.start(); nxt = 0
try:
    while time.monotonic() - t0 < END:
        now = time.monotonic() - t0
        s.set_controls(Controls())
        if nxt < len(PLAN) and now >= starts[nxt]:
            bell(f"Sweep {PLAN[nxt][0]} SLOWLY (about 60 s) {PLAN[nxt][1]}, flat object, level"); nxt += 1
        time.sleep(0.01)
finally:
    s.set_controls(Controls()); time.sleep(0.1); s.stop()
bell("Sweep test finished")
L = []
dists = []
for t, p in d3:
    q = p[2:] if (len(p) == 12 and p[0] == 0 and p[1] == 0xC8) else p
    if len(q) >= 2: dists.append((t, int.from_bytes(q[0:2], "big"), len(p), p[:2].hex()))
L.append(f"range frames: {len(dists)}; payload lengths {sorted({x[2] for x in dists})}; prefixes {sorted({x[3] for x in dists})}")
for mm, who in RISKY.items():
    near = [x for x in dists if x[1] == mm]
    L.append(f"samples at exactly {mm} mm ({who}): {len(near)}")
L.append(f"distance range seen: {min(x[1] for x in dists) if dists else None} - {max(x[1] for x in dists) if dists else None} mm")
# TX health per second, from link-up to the very end
first_fc = min((sec for sec, c in per_sec.items() if c[0x1E] or c[0x08]), default=None)
bad = []
if first_fc is not None:
    for sec in range(first_fc + 2, int(END)):
        c = per_sec.get(sec, collections.Counter())
        if c[0x3A] == 0 or (c[0x1E] + c[0x08] + c[0x21]) == 0:
            bad.append(sec)
L.append(f"link-up (first FC frame) at {first_fc} s; seconds without sync or FC telemetry afterwards: {len(bad)} {bad[:15]}")
L.append("RESULT: " + ("TX stayed up through the whole sweep" if (first_fc is not None and not bad) else "TX/link problem, see the frame log"))
open(OUT, "w").write("\n".join(L) + "\n")
with open(OUT + ".frames.txt", "w") as f:
    for sec in sorted(per_sec): f.write(f"{sec:4d} " + " ".join(f"{k:02x}:{v}" for k, v in sorted(per_sec[sec].items())) + "\n")
    for t, p in d3: f.write(f"D3 {t:.3f} {p.hex()}\n")
print("\n".join(L))
