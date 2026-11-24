#!/usr/bin/env python3
"""Set exactly one Betaflight PID-profile parameter, iterm_limit (cap on the attitude integrator, default 400), save, and report before/after. Refuses if the FC is armed.
Only 'get iterm_limit', 'set iterm_limit = <n>' and 'save' are ever sent.   usage: fc_set_iterm_limit.py <fc usb port> <value 20..400>"""
import os, select, struct, sys, termios, time
port, new = sys.argv[1], int(sys.argv[2])
assert 20 <= new <= 400, "iterm_limit out of the sane range"
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
i = buf.find(b"$M>"); payload = buf[i + 5:i + 5 + buf[i + 3]] if i >= 0 else b""
flags = struct.unpack_from("<I", payload, 6)[0] if len(payload) >= 10 else None
print("mode flags:", None if flags is None else hex(flags))
if flags is None or flags & 1: print("REFUSING: FC armed or state unknown"); sys.exit(1)
os.write(fd, b"#"); drain()
os.write(fd, b"get iterm_limit\r\n"); print("before:", " | ".join(x.strip() for x in drain().splitlines() if "iterm_limit" in x and "get" not in x))
os.write(fd, f"set iterm_limit = {new}\r\n".encode()); print("set   :", " ".join(x.strip() for x in drain().splitlines() if x.strip())[:120])
os.write(fd, b"get iterm_limit\r\n"); print("after :", " | ".join(x.strip() for x in drain().splitlines() if "iterm_limit" in x and "get" not in x))
os.write(fd, b"save\r\n"); drain(0.5, 3); os.close(fd); print("saved (FC reboots)")
