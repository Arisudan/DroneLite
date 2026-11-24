#!/usr/bin/env python3
"""Read raw MTF-02P output through Betaflight serialpassthrough (UART3 -> USB).
Only these CLI commands are sent: '#' (enter CLI) and 'serialpassthrough 2 115200'.
usage: mtf02p_passthrough_read_raw.py <port> <raw_out.bin>
The FC stays in passthrough until power-cycled.
"""
import os
import select
import struct
import subprocess
import sys
import termios
import time

port, raw_out = sys.argv[1:3]
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


def say(t):
    subprocess.Popen(["spd-say", "-w", t], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.Popen(["notify-send", "-t", "4000", "MTF-02P passthrough", t], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


CMD = b"serialpassthrough 2 115200\r\n"
os.write(fd, b"#")
drain()
os.write(fd, CMD)
print("CLI reply:", drain(q=1.0, lim=4).decode(errors="replace").strip().replace("\r", " ")[:200])

phases = [("COUNTDOWN", 5, "Passthrough test starts in five seconds. Sensor facing the table, hold still."),
          ("STILL", 8, "Hold still, fifteen centimeters above the table."),
          ("RAISE", 12, "Raise slowly to forty centimeters, then lower. Repeat."),
          ("HOLD_HIGH", 7, "Hold steady at about thirty centimeters."),
          ("COVER", 6, "Cover the sensor with your hand, almost touching.")]
chunks = []  # (phase, time, bytes)
for name, dur, cue in phases:
    say(cue)
    end = time.time() + dur
    while time.time() < end:
        if select.select([fd], [], [], 0.05)[0]:
            d = os.read(fd, 4096)
            if d:
                chunks.append((name, time.time(), d))
say("Test finished. Please power cycle the flight controller.")
os.close(fd)
open(raw_out, "wb").write(b"".join(c[2] for c in chunks))
total = sum(len(c[2]) for c in chunks)
print("raw bytes captured:", total)
import pickle
pickle.dump(chunks, open(raw_out + ".chunks", "wb"))
