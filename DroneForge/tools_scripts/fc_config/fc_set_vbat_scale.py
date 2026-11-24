#!/usr/bin/env python3
"""Set Betaflight vbat_scale (battery voltage calibration), save, and report the before value. Refuses if the FC is armed.
The new value must be calibrated against a multimeter: new = old * (real volts / FC reading).
usage: fc_set_vbat_scale.py <port> <new_scale 20..255> [new_multiplier 1..10] [new_divider 1..50]
Betaflight: reading is proportional to vbat_scale / (vbat_divider * vbat_multiplier) (verified on this FC 2026-11-03).
vbat_scale alone cannot exceed 255, so a big gain correction lowers the divider."""
import os, re, select, struct, sys, termios, time
port, new = sys.argv[1], int(sys.argv[2])
mult = int(sys.argv[3]) if len(sys.argv) > 3 else None
div = int(sys.argv[4]) if len(sys.argv) > 4 else None
assert 20 <= new <= 255, "vbat_scale out of the sane range"
assert mult is None or 1 <= mult <= 10, "vbat_multiplier out of the sane range"
assert div is None or 1 <= div <= 50, "vbat_divider out of the sane range"
fd = os.open(port, os.O_RDWR | os.O_NOCTTY)
a = termios.tcgetattr(fd); a[0]=a[1]=a[3]=0; a[2]=termios.CS8|termios.CREAD|termios.CLOCAL; a[4]=a[5]=termios.B115200
a[6][termios.VMIN], a[6][termios.VTIME] = 0, 1
termios.tcsetattr(fd, termios.TCSANOW, a); termios.tcflush(fd, termios.TCIOFLUSH)
def drain(q=0.7, lim=6):
    buf, last, end = b"", time.time(), time.time() + lim
    while time.time() < end:
        if select.select([fd], [], [], 0.1)[0]:
            d = os.read(fd, 4096)
            if d: buf += d; last = time.time()
        elif time.time() - last > q: break
    return buf.decode(errors="replace")
os.write(fd, b"$M<" + bytes([0, 101, 101])); buf, end = b"", time.time() + 1.0
while time.time() < end:
    if select.select([fd], [], [], 0.05)[0]: buf += os.read(fd, 512)
    i = buf.find(b"$M>")
    if i >= 0 and len(buf) >= i + 6 and len(buf) >= i + 6 + buf[i + 3]: break
i = buf.find(b"$M>"); p = buf[i + 5:i + 5 + buf[i + 3]] if i >= 0 else b""
flags = struct.unpack_from("<I", p, 6)[0] if len(p) >= 10 else None
if flags is None or flags & 1:
    sys.exit("REFUSING: FC armed or state unknown")
os.write(fd, b"#"); drain()
os.write(fd, b"get vbat_scale\r\n"); out = drain(q=0.5, lim=3)
m = re.search(r"vbat_scale\s*=\s*(\d+)", out); old = int(m.group(1)) if m else None
os.write(fd, b"get vbat_multiplier\r\n"); out2 = drain(q=0.5, lim=3)
m2 = re.search(r"vbat_multiplier\s*=\s*(\d+)", out2)
print("before: vbat_scale =", old, "| vbat_multiplier =", m2.group(1) if m2 else None)
os.write(fd, f"set vbat_scale = {new}\r\n".encode()); out = drain(q=0.5, lim=3)
print("set =>", " ".join(x.strip() for x in out.splitlines() if x.strip())[:100])
if mult is not None:
    os.write(fd, f"set vbat_multiplier = {mult}\r\n".encode()); drain(q=0.5, lim=3)
if div is not None:
    os.write(fd, f"set vbat_divider = {div}\r\n".encode()); drain(q=0.5, lim=3)
os.write(fd, b"get vbat_scale\r\n"); out = drain(q=0.5, lim=3)
m = re.search(r"vbat_scale\s*=\s*(\d+)", out)
os.write(fd, b"get vbat_multiplier\r\n"); out3 = drain(q=0.5, lim=3)
m3 = re.search(r"vbat_multiplier\s*=\s*(\d+)", out3)
os.write(fd, b"get vbat_divider\r\n"); out4 = drain(q=0.5, lim=3)
m4 = re.search(r"vbat_divider\s*=\s*(\d+)", out4)
print("readback before save: vbat_scale =", m.group(1) if m else None, "| vbat_multiplier =", m3.group(1) if m3 else None, "| vbat_divider =", m4.group(1) if m4 else None)
os.write(fd, b"save\r\n"); drain(q=0.5, lim=3); os.close(fd)
print("saved (FC reboots)")
