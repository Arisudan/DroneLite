#!/usr/bin/env python3
"""Apply an exact whitelist of Betaflight CLI lines, then save. Refuses if the FC is armed.
usage: fc_apply_cli_lines.py <port>
"""
import os, select, struct, sys, termios, time
LINES = [
    "aux 5 40 4 1100 2100 0 0",        # USER1 on AUX5 (CH9) above 1100 us = FC height hold request (needs the height-hold firmware)
]
port = sys.argv[1]
fd = os.open(port, os.O_RDWR | os.O_NOCTTY)
a = termios.tcgetattr(fd); a[0]=a[1]=a[3]=0; a[2]=termios.CS8|termios.CREAD|termios.CLOCAL; a[4]=a[5]=termios.B115200
a[6][termios.VMIN], a[6][termios.VTIME] = 0, 1
termios.tcsetattr(fd, termios.TCSANOW, a); termios.tcflush(fd, termios.TCIOFLUSH)
def drain(q=0.7, lim=8):
    buf, last, end = b"", time.time(), time.time() + lim
    while time.time() < end:
        if select.select([fd], [], [], 0.1)[0]:
            d = os.read(fd, 4096)
            if d: buf += d; last = time.time()
        elif time.time() - last > q: break
    return buf.decode(errors="replace")
# safety: refuse if armed (MSP_STATUS flags bit0 = ARM)
os.write(fd, b"$M<" + bytes([0, 101, 101])); buf, end = b"", time.time() + 1.0
while time.time() < end:
    if select.select([fd], [], [], 0.05)[0]: buf += os.read(fd, 512)
    i = buf.find(b"$M>")
    if i >= 0 and len(buf) >= i + 6 and len(buf) >= i + 6 + buf[i + 3]: break
i = buf.find(b"$M>")
payload = buf[i + 5:i + 5 + buf[i + 3]] if i >= 0 else b""
flags = struct.unpack_from("<I", payload, 6)[0] if len(payload) >= 10 else None
print("mode flags:", None if flags is None else hex(flags))
if flags is None or flags & 1:
    print("REFUSING: FC armed or state unknown"); sys.exit(1)
os.write(fd, b"#"); drain()
for l in LINES:
    assert l in LINES
    os.write(fd, l.encode() + b"\r\n")
    out = drain(q=0.5, lim=4)
    print(">", l, "=>", " ".join(x.strip() for x in out.splitlines() if x.strip() and x.strip() != l)[:120])
os.write(fd, b"save\r\n"); drain(q=0.5, lim=3)
os.close(fd); print("saved (FC reboots)")
