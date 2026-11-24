#!/usr/bin/env python3
"""Accelerometer LEVEL calibration of the FC (writes!). DRONE ON A CHECKED-LEVEL SURFACE, STILL, DISARMED, BATTERY OFF, FC ON USB.
Steps: (1) read pitch/roll before (MSP_ATTITUDE, read-only) (2) CLI: acc_trim_roll/pitch = 0, save (FC reboots) (3) MSP_ACC_CALIBRATION (205), wait (4) read pitch/roll after.
Refuses if the FC reports armed or the drone is moving. usage: fc_calibrate_accel_level_usb.py <fc usb port> [--yes]"""
import os, select, struct, sys, termios, time, statistics as st
port = sys.argv[1]

def opn(path):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
    a = termios.tcgetattr(fd); a[0] = a[1] = a[3] = 0; a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL; a[4] = a[5] = termios.B115200
    a[6][termios.VMIN], a[6][termios.VTIME] = 0, 0
    termios.tcsetattr(fd, termios.TCSANOW, a); termios.tcflush(fd, termios.TCIOFLUSH); return fd

def msp(fd, cmd, timeout=0.4):
    assert cmd in (101, 108, 205), "only status, attitude and accelerometer calibration are ever sent"
    os.write(fd, b"$M<" + bytes([0, cmd, cmd])); buf, end = b"", time.time() + timeout
    while time.time() < end:
        if select.select([fd], [], [], 0.02)[0]: buf += os.read(fd, 512)
        i = buf.find(b"$M>")
        if i >= 0 and len(buf) >= i + 6 and len(buf) >= i + 6 + buf[i + 3]: return buf[i + 5:i + 5 + buf[i + 3]]
    return None

def attitude(fd, n=30):
    r, p = [], []
    for _ in range(n):
        a = msp(fd, 108)
        if a and len(a) >= 4: r.append(struct.unpack_from("<h", a, 0)[0] / 10.0); p.append(struct.unpack_from("<h", a, 2)[0] / 10.0)
        time.sleep(0.05)
    return (st.median(r), st.median(p), max(r) - min(r), max(p) - min(p)) if r else None

def armed(fd):
    s = msp(fd, 101); return None if not s or len(s) < 10 else bool(struct.unpack_from("<I", s, 6)[0] & 1)

def cli(fd, lines, quiet=0.5):
    os.write(fd, b"#\r\n"); time.sleep(0.6); out = b""
    for l in lines:
        os.write(fd, l.encode() + b"\r\n"); end = time.time() + 1.5; last = time.time()
        while time.time() < end:
            if select.select([fd], [], [], 0.1)[0]: out += os.read(fd, 4096); last = time.time()
            elif time.time() - last > quiet: break
    return out.decode(errors="replace")

fd = opn(port)
if armed(fd) is not False: sys.exit("REFUSING: FC armed or not answering")
b = attitude(fd)
print("BEFORE: roll %+.1f pitch %+.1f deg (noise span roll %.1f pitch %.1f)" % b)
if b[2] > 0.6 or b[3] > 0.6: sys.exit("REFUSING: the drone is moving (span > 0.6 deg). Keep it still.")
if "--yes" not in sys.argv: sys.exit("dry run: add --yes to write")
out = cli(fd, ["set acc_trim_roll = 0", "set acc_trim_pitch = 0", "save"], 0.6)
print("trims ->", [l.strip() for l in out.splitlines() if "acc_trim" in l and "set" not in l][:2], "| saved (FC reboots)")
try: os.close(fd)
except OSError: pass
time.sleep(6)
for _ in range(20):
    if os.path.exists(port): break
    time.sleep(1)
time.sleep(3); fd = opn(port)
if armed(fd) is not False: sys.exit("REFUSING calibration: FC armed or not answering after the reboot")
print("calibrating the accelerometer (keep the drone still) ..."); msp(fd, 205); time.sleep(8)
a = attitude(fd)
print("AFTER:  roll %+.1f pitch %+.1f deg (noise span roll %.1f pitch %.1f)" % a)
print("RESULT:", "LEVEL (within 0.3 deg)" if abs(a[0]) <= 0.3 and abs(a[1]) <= 0.3 else "NOT within 0.3 deg: check the surface is level; calibration done anyway")
os.close(fd)
