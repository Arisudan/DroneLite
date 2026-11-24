#!/usr/bin/env python3
"""Does the TX stop forwarding right after a 0xD4 (motor/raw) frame whose first two payload bytes look like CRSF device addresses?
ELRS 4.x reads frame types >= 0x28 as 'extended': payload[0] = destination, payload[1] = origin, and the router learns origins.
DISARMED, PROPS OFF, drone stays ON THE TABLE; you tilt/shake it gently by hand. Only the adapter is used.
usage: bench_test_d4_frame_collision.py <adapter_port> <lead_s> <window_s> <out_file>"""
import collections, subprocess, sys, time
sys.path.insert(0, "/home/htic/Music/DroneForge/software/pc")
from nimbus_pc.sender import Controls
from nimbus_pc.session import Session

ADP, LEAD, WIN, OUT = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), sys.argv[4]
FC_TYPES = {0x08, 0x1E, 0x21, 0x09, 0x07, 0xD3, 0xD4}
ADDR = {0xEA: "radio(EA)", 0xEE: "TX(EE)", 0xEC: "RX(EC)", 0xC8: "FC(C8)", 0xEF: "Lua(EF)", 0x00: "broadcast(00)"}
t0 = time.monotonic()
per_sec = collections.defaultdict(collections.Counter)
d4 = []


def on_frame(f):
    t = time.monotonic() - t0
    per_sec[int(t)][f.type] += 1
    if f.type == 0xD4 and len(f.payload) >= 2:
        d4.append((t, f.payload[0], f.payload[1]))


def bell(msg):
    subprocess.Popen(["pw-play", "/usr/share/sounds/freedesktop/stereo/bell.oga"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.Popen(["notify-send", "-u", "critical", "-t", "7000", "D4 test", msg], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


s = Session(ADP, hz=100, allow_arm=False, ping=True, on_frame=on_frame)
s.start(); rang = [False, False]
try:
    while time.monotonic() - t0 < LEAD + WIN + 3:
        t = time.monotonic() - t0
        s.set_controls(Controls())
        if t >= LEAD and not rang[0]:
            rang[0] = True; bell("GO: gently tilt / rock / shake the drone on the table (props off)")
        if t >= LEAD + WIN and not rang[1]:
            rang[1] = True; bell("STOP: put the drone down flat and still")
        time.sleep(0.01)
finally:
    s.stop()

w_ = []
w = w_.append
b0, b1 = collections.Counter(x[1] for x in d4), collections.Counter(x[2] for x in d4)
w(f"0xD4 frames: {len(d4)}; distinct byte0 values {len(b0)}, distinct byte1 values {len(b1)}")
w("byte1 (ELRS 'origin') equal to a real device address: " + str({ADDR[k]: v for k, v in b1.items() if k in ADDR}))
w("byte0 (ELRS 'destination') equal to a real device address: " + str({ADDR[k]: v for k, v in b0.items() if k in ADDR}))
stuck_from = None
for sec in range(0, int(LEAD + WIN + 3)):
    c = per_sec.get(sec, collections.Counter())
    n14, n3a, nfc = c[0x14], c[0x3A], sum(c[x] for x in FC_TYPES)
    if n14 > 0 and n3a == 0 and nfc == 0 and sec > 8 and stuck_from is None:
        stuck_from = sec
if stuck_from is None:
    w("RESULT: the TX never got stuck.")
else:
    hits = [(round(t, 1), hex(a), hex(b)) for t, a, b in d4 if stuck_from - 3 <= t <= stuck_from + 0.5]
    w(f"RESULT: the TX got STUCK at about {stuck_from} s. Last 0xD4 frames before it (t, byte0, byte1): {hits[-10:]}")
    w("address-like byte among them: " + str([(t, a, b) for t, a, b in hits if int(a, 16) in ADDR or int(b, 16) in ADDR]))
open(OUT, "w").write("\n".join(w_) + "\n"); print("\n".join(w_))
