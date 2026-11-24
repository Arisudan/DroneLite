#!/usr/bin/env python3
"""Does the TX stop forwarding when the lidar range hits a value whose low byte is 0xEA (234, 490, 746 mm...)?
ELRS reads frame types >= 0x28 as 'extended': the first two payload bytes become destination/origin addresses. Our 0xD3 frame starts
with the distance in mm (big endian), so its low byte lands in the origin-address slot. DISARMED, PROPS OFF; only the adapter is used.
usage: bench_test_lidar_range_stuck_value.py <adapter_port> <lead_s> <window_s> <out_file>
A bell rings at lead_s (start moving an object over the sensor, 5 cm -> 80 cm and back, slowly) and at lead_s + window_s (stop)."""
import collections, struct, subprocess, sys, time
sys.path.insert(0, "/home/htic/Music/DroneForge/software/pc")
from nimbus_pc.sender import Controls
from nimbus_pc.session import Session

ADP, LEAD, WIN, OUT = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), sys.argv[4]
FC_TYPES = {0x08, 0x1E, 0x21, 0x09, 0x07, 0xD3, 0xD4}
t0 = time.monotonic()
per_sec = collections.defaultdict(collections.Counter)
dist = []


def on_frame(f):
    t = time.monotonic() - t0
    per_sec[int(t)][f.type] += 1
    if f.type == 0xD3 and len(f.payload) >= 2:
        dist.append((t, struct.unpack(">h", bytes(f.payload[:2]))[0]))


def bell(msg):
    subprocess.Popen(["pw-play", "/usr/share/sounds/freedesktop/stereo/bell.oga"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.Popen(["notify-send", "-u", "critical", "-t", "7000", "Range test", msg], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


s = Session(ADP, hz=100, allow_arm=False, ping=True, on_frame=on_frame)
s.start()
rang = [False, False]
try:
    while time.monotonic() - t0 < LEAD + WIN + 3:
        t = time.monotonic() - t0
        s.set_controls(Controls())
        if t >= LEAD and not rang[0]:
            rang[0] = True; bell("GO: slowly move the object 5 cm -> 80 cm -> 5 cm over the sensor, repeat")
        if t >= LEAD + WIN and not rang[1]:
            rang[1] = True; bell("STOP: take the object away")
        time.sleep(0.01)
finally:
    s.stop()

lines = []
w = lines.append
w(f"range samples: {len(dist)}  min {min((m for _, m in dist), default=None)} mm  max {max((m for _, m in dist), default=None)} mm")
hits = [(t, m) for t, m in dist if (m & 0xFF) == 0xEA]
w(f"samples whose low byte is 0xEA (the radio's address): {len(hits)}  e.g. {[m for _, m in hits[:8]]}")
w("per second: t | link(0x14) sync(0x3A) fw-telemetry ping-reply(0x29) | range mm (last in that second)")
stuck_from = None
last_mm = {}
for t, m in dist:
    last_mm[int(t)] = m
for sec in range(int(t0 * 0), int(LEAD + WIN + 3)):
    c = per_sec.get(sec, collections.Counter())
    n14, n3a, nfc, n29 = c[0x14], c[0x3A], sum(c[x] for x in FC_TYPES), c[0x29]
    stuck = n14 > 0 and n3a == 0 and nfc == 0 and sec > 8
    if stuck and stuck_from is None:
        stuck_from = sec
    mark = "  <== STUCK (only link stats)" if stuck else ""
    w(f"{sec:3d}s | {n14:2d} {n3a:3d} {nfc:3d} {n29:2d} | {last_mm.get(sec, '--')}{mark}")
w("")
if stuck_from is None:
    w("RESULT: the TX never got stuck during the test.")
else:
    before = [(round(t, 1), m) for t, m in dist if stuck_from - 3 <= t <= stuck_from + 0.5]
    w(f"RESULT: the TX got STUCK at about {stuck_from} s. Range readings in the 3 s before: {before[-12:]}")
    w("0xEA low byte among them: " + str([m for _, m in before if (m & 0xFF) == 0xEA]))
open(OUT, "w").write("\n".join(lines) + "\n")
print("\n".join(lines))
