#!/usr/bin/env python3
"""PC -> sender -> TX -> RF -> RX -> FC control latency, disarmed. Steps the ROLL channel and times when the FC (read over USB MSP_RC)
first shows the change. Includes the sender's 10 ms frame period. Accuracy about +-10 ms (MSP polling).
usage: bench_measure_pc_to_fc_latency.py <fc_port> <adapter_port> [flips]"""
import os, select, statistics as st, struct, sys, termios, time
sys.path.insert(0, "/home/htic/Music/DroneForge/software/pc")
from nimbus_pc.sender import Controls
from nimbus_pc.session import Session

FC, ADP = sys.argv[1], sys.argv[2]
FLIPS = int(sys.argv[3]) if len(sys.argv) > 3 else 30
fd = os.open(FC, os.O_RDWR | os.O_NOCTTY)
a = termios.tcgetattr(fd); a[0] = a[1] = a[3] = 0; a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL; a[4] = a[5] = termios.B115200
a[6][termios.VMIN], a[6][termios.VTIME] = 0, 1
termios.tcsetattr(fd, termios.TCSANOW, a); termios.tcflush(fd, termios.TCIOFLUSH)


def roll_us():
    os.write(fd, b"$M<" + bytes([0, 105, 105])); buf, end = b"", time.monotonic() + 0.1
    while time.monotonic() < end:
        if select.select([fd], [], [], 0.002)[0]: buf += os.read(fd, 512)
        i = buf.find(b"$M>")
        if i >= 0 and len(buf) >= i + 6 and len(buf) >= i + 6 + buf[i + 3]:
            p = buf[i + 5:i + 5 + buf[i + 3]]; return struct.unpack_from("<H", p, 0)[0]


s = Session(ADP, hz=100, allow_arm=False, ping=False)
s.start()
lat, polls = [], []
try:
    s.set_controls(Controls()); t_end = time.monotonic() + 6
    while time.monotonic() < t_end:           # let the link settle
        s.set_controls(Controls()); roll_us(); time.sleep(0.01)
    high = False
    for i in range(FLIPS):
        high = not high
        want = 1700 if high else 1300
        t0 = time.monotonic(); s.set_controls(Controls(roll=0.7 if high else -0.7))
        hit = None
        while time.monotonic() - t0 < 1.5:
            t1 = time.monotonic(); r = roll_us(); polls.append(time.monotonic() - t1)
            if r is not None and ((high and r > 1600) or (not high and r < 1400)):
                hit = time.monotonic() - t0; break
        if hit is not None: lat.append(hit * 1000)
        time.sleep(0.4)
finally:
    s.set_controls(Controls()); time.sleep(0.3); s.stop()
if lat:
    lat.sort()
    print(f"flips measured {len(lat)}/{FLIPS}: latency ms  min {lat[0]:.0f}  median {st.median(lat):.0f}  mean {st.mean(lat):.0f}  p90 {lat[int(len(lat)*.9)-1]:.0f}  max {lat[-1]:.0f}")
    print(f"MSP poll time median {st.median(polls)*1000:.0f} ms (the measurement's own resolution)")
else:
    print("no flips detected: link down or FC not receiving")
