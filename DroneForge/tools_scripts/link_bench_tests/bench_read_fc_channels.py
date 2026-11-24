"""Print the RC channels the FC receives (MSP_RC, read-only). usage: <fc_port> <seconds>"""
import os, select, struct, sys, termios, time
FC = sys.argv[1]; dur = float(sys.argv[2])
fd = os.open(FC, os.O_RDWR | os.O_NOCTTY)
a = termios.tcgetattr(fd); a[0] = a[1] = a[3] = 0; a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL; a[4] = a[5] = termios.B115200
a[6][termios.VMIN], a[6][termios.VTIME] = 0, 1
termios.tcsetattr(fd, termios.TCSANOW, a); termios.tcflush(fd, termios.TCIOFLUSH)
def msp(cmd):                      # read-only requests only
    assert cmd in (105, 110)
    os.write(fd, b"$M<" + bytes([0, cmd, cmd])); buf, end = b"", time.time() + 0.15
    while time.time() < end:
        if select.select([fd], [], [], 0.02)[0]: buf += os.read(fd, 512)
        i = buf.find(b"$M>")
        if i >= 0 and len(buf) >= i + 6 and len(buf) >= i + 6 + buf[i + 3]: return buf[i + 5:i + 5 + buf[i + 3]]
rows = []; t0 = time.time()
while time.time() - t0 < dur:
    rc = msp(105)
    if rc: rows.append(struct.unpack(f"<{len(rc)//2}H", rc))
    time.sleep(0.08)
print("samples:", len(rows))
if rows:
    for c in range(min(len(rows[0]), 8)):
        v = [r[c] for r in rows]; print(f"ch{c+1}: min {min(v)} max {max(v)} last {v[-1]}")
