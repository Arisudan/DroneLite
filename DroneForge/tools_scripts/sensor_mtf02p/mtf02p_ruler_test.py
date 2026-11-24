#!/usr/bin/env python3
"""Ruler test of the MTF-02P through Betaflight serialpassthrough (UART3 -> USB).
Sends only '#' and 'serialpassthrough 2 115200'. FC stays in passthrough until power-cycled.
usage: mtf02p_ruler_test.py <port>
"""
import os
import select
import statistics as st
import struct
import subprocess
import sys
import termios
import time

port = sys.argv[1]
fd = os.open(port, os.O_RDWR | os.O_NOCTTY)
a = termios.tcgetattr(fd)
a[0] = a[1] = a[3] = 0
a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
a[4] = a[5] = termios.B115200
a[6][termios.VMIN], a[6][termios.VTIME] = 0, 1
termios.tcsetattr(fd, termios.TCSANOW, a)


def drain(q=0.8, lim=6):
    buf, last, end = b"", time.time(), time.time() + lim
    while time.time() < end:
        if select.select([fd], [], [], 0.1)[0]:
            d = os.read(fd, 4096)
            if d:
                buf += d
                last = time.time()
        elif time.time() - last > q:
            break
    return buf


BELL = "/usr/share/sounds/freedesktop/stereo/bell.oga"


def cue(text):
    subprocess.Popen(["pw-play", BELL], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.Popen(["spd-say", "-w", text], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.Popen(["notify-send", "-u", "critical", "-t", "6000", "MTF-02P ruler test", text],
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


os.write(fd, b"#")
drain()
os.write(fd, b"serialpassthrough 2 115200\r\n")
print("CLI:", drain(q=1.0, lim=4).decode(errors="replace").strip().replace("\r", " ").split("Forwarding")[-1][:60])

# (label, expected_cm or None, seconds, cue)
phases = [("SETUP", None, 8, "Ruler test starts in eight seconds. Put the drone resting on the table, sensor facing down."),
          ("REST", None, 10, "Resting on the table. Do not touch it."),
          ("H10", 10, 9, "Lift. Sensor ten centimeters above the table. Hold steady."),
          ("H20", 20, 9, "Twenty centimeters. Hold steady."),
          ("H30", 30, 9, "Thirty centimeters. Hold steady."),
          ("H40", 40, 9, "Forty centimeters. Hold steady."),
          ("REST2", None, 8, "Put it back on the table and let go.")]
chunks = []
for name, exp, dur, text in phases:
    cue(text)
    end = time.time() + dur
    while time.time() < end:
        if select.select([fd], [], [], 0.05)[0]:
            d = os.read(fd, 4096)
            if d:
                chunks.append((name, time.time(), d))
cue("Test finished. Please power cycle the flight controller now.")
os.close(fd)

frames, carry = [], b""
for phase, t, d in chunks:
    buf = carry + d
    j = 0
    while True:
        j = buf.find(b"\xef", j)
        if j < 0 or j + 7 > len(buf):
            carry = buf[j:] if j >= 0 else b""
            break
        ln = buf[j + 5]
        end = j + 6 + ln + 1
        if ln > 64:
            j += 1
            continue
        if end > len(buf):
            carry = buf[j:]
            break
        if (sum(buf[j:end - 1]) & 0xFF) == buf[end - 1] and buf[j + 3] == 0x51 and ln == 20:
            frames.append((phase, t, buf[j + 6:j + 26]))
            j = end
        else:
            j += 1

print(f"frames decoded: {len(frames)}")
print(f"{'phase':6s} {'ruler cm':>8s} | last-4s distance mm: {'min':>5s} {'median':>6s} {'max':>5s} {'sd':>5s} | flow quality | tof status | note")
for name, exp, dur, _ in phases:
    fs = [f for f in frames if f[0] == name]
    if not fs:
        print(f"{name:6s} no frames")
        continue
    t_end = fs[-1][1]
    last = [f for f in fs if f[1] >= t_end - 4.0]
    dist = [struct.unpack_from("<I", p, 4)[0] for _, _, p in last]
    fq = int(st.median(p[16] for _, _, p in last))
    tof = sorted({p[10] for _, _, p in last})
    med = int(st.median(dist))
    note = ""
    if exp is not None:
        err = med - exp * 10
        note = f"error vs ruler {err:+d} mm ({100.0 * err / (exp * 10):+.0f}%)"
    print(f"{name:6s} {str(exp) if exp else '-':>8s} | {min(dist):26d} {med:6d} {max(dist):5d} {st.pstdev(dist):5.1f} | {fq:12d} | {str(tof):10s} | {note}")
