#!/usr/bin/env python3
"""Background recorder: enter serialpassthrough once, then log MTF-02P frames to CSV until killed or timeout.
Sends only '#' and 'serialpassthrough 2 115200'.
usage: mtf02p_record_csv_background.py <port> <out.csv> [max_seconds]
"""
import os, select, struct, sys, termios, time
port, out = sys.argv[1], sys.argv[2]
maxs = float(sys.argv[3]) if len(sys.argv) > 3 else 1500
fd = os.open(port, os.O_RDWR | os.O_NOCTTY)
a = termios.tcgetattr(fd); a[0]=a[1]=a[3]=0; a[2]=termios.CS8|termios.CREAD|termios.CLOCAL; a[4]=a[5]=termios.B115200
a[6][termios.VMIN], a[6][termios.VTIME] = 0, 1
termios.tcsetattr(fd, termios.TCSANOW, a)
def drain(q=0.8, lim=5):
    buf, last, end = b"", time.time(), time.time() + lim
    while time.time() < end:
        if select.select([fd], [], [], 0.1)[0]:
            d = os.read(fd, 4096)
            if d: buf += d; last = time.time()
        elif time.time() - last > q: break
    return buf
os.write(fd, b"#"); drain()
os.write(fd, b"serialpassthrough 2 115200\r\n"); drain(q=1.0, lim=4)
f = open(out, "w"); f.write("t,dist_mm,strength,tof,flow_q,flow_x,flow_y\n"); f.flush()
carry, t0, last_flush = b"", time.time(), time.time()
while time.time() - t0 < maxs:
    if select.select([fd], [], [], 0.1)[0]:
        d = os.read(fd, 4096); t = time.time()
        buf = carry + d; j = 0
        while True:
            j = buf.find(b"\xef", j)
            if j < 0 or j + 7 > len(buf): carry = buf[j:] if j >= 0 else b""; break
            ln = buf[j+5]; end = j + 6 + ln + 1
            if ln > 64: j += 1; continue
            if end > len(buf): carry = buf[j:]; break
            if (sum(buf[j:end-1]) & 0xFF) == buf[end-1] and buf[j+3] == 0x51 and ln == 20:
                p = buf[j+6:j+26]
                f.write(f"{t:.3f},{struct.unpack_from('<I', p, 4)[0]},{p[8]},{p[10]},{p[16]},{struct.unpack_from('<h', p, 12)[0]},{struct.unpack_from('<h', p, 14)[0]}\n")
                j = end
            else: j += 1
    if time.time() - last_flush > 0.5: f.flush(); last_flush = time.time()
