#!/usr/bin/env python3
"""Read-only full Betaflight configuration dump (CLI `dump all`, `version`, `rateprofile`/`profile` selection is only READ via the dump).
Never sets, saves or exits (an `exit` drops this FC off USB until a power cycle): the FC stays in CLI mode until it is power cycled, so power-cycle it afterwards.
usage: fc_dump_all_cli_readonly.py <fc usb port> <out file>"""
import os, select, sys, termios, time
ALLOWED = ("version", "dump all", "status", "tasks", "get gyro_lpf1_static_hz")
fd = os.open(sys.argv[1], os.O_RDWR | os.O_NOCTTY)
a = termios.tcgetattr(fd); a[0] = a[1] = a[3] = 0; a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL; a[4] = a[5] = termios.B115200
a[6][termios.VMIN], a[6][termios.VTIME] = 0, 1
termios.tcsetattr(fd, termios.TCSANOW, a)
def drain(quiet=1.5, limit=60):
    buf, last, end = b"", time.time(), time.time() + limit
    while time.time() < end:
        if select.select([fd], [], [], 0.1)[0]:
            d = os.read(fd, 8192)
            if d: buf += d; last = time.time()
        elif time.time() - last > quiet: break
    return buf.decode(errors="replace")
os.write(fd, b"#"); out = [drain(0.8, 6)]
for c in ("version", "dump all"):
    assert c in ALLOWED
    os.write(fd, c.encode() + b"\r\n"); out.append(f"\n>>> {c}\n" + drain(2.5 if c == "dump all" else 0.8))
open(sys.argv[2], "w").write("".join(out)); os.close(fd)
print("saved", sys.argv[2], len("".join(out)), "bytes")
