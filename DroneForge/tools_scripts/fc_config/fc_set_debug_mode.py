#!/usr/bin/env python3
"""Set exactly one Betaflight parameter (debug_mode) via CLI and save.
usage: fc_set_debug_mode.py <port> <RANGEFINDER|OPTICALFLOW|DF_HOLD|NONE>   (DF_HOLD: firmware v4.7.4R and later)
Only 'get debug_mode', 'set debug_mode = <value>' and 'save' are ever sent.
"""
import os, select, sys, termios, time
port, value = sys.argv[1:3]
assert value in ("RANGEFINDER", "OPTICALFLOW", "DF_HOLD", "NONE")
fd = os.open(port, os.O_RDWR | os.O_NOCTTY)
a = termios.tcgetattr(fd); a[0]=a[1]=a[3]=0; a[2]=termios.CS8|termios.CREAD|termios.CLOCAL; a[4]=a[5]=termios.B115200
a[6][termios.VMIN], a[6][termios.VTIME] = 0, 1
termios.tcsetattr(fd, termios.TCSANOW, a)
def drain(q=0.7, lim=10):
    buf, last, end = b"", time.time(), time.time() + lim
    while time.time() < end:
        if select.select([fd], [], [], 0.1)[0]:
            d = os.read(fd, 4096)
            if d: buf += d; last = time.time()
        elif time.time() - last > q: break
    return buf.decode(errors="replace")
def send(c):
    assert c in ("get debug_mode", f"set debug_mode = {value}", "save"), c
    os.write(fd, c.encode() + b"\r\n")
    return drain()
os.write(fd, b"#"); drain()
print("BEFORE:", [l for l in send("get debug_mode").splitlines() if "debug_mode =" in l])
out = send(f"set debug_mode = {value}")
print("SET   :", [l.strip() for l in out.splitlines() if "debug_mode" in l or "Invalid" in l])
send("save")   # saves and reboots the FC
os.close(fd)
