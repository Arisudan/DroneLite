#!/usr/bin/env python3
"""READ-ONLY failsafe/arming monitor: MSP_STATUS(101), MSP_MOTOR(104), MSP_RC(105), MSP_ANALOG(110).
Never sends any command that can change state or move motors.
usage: link_monitor_failsafe_motors_readonly.py <fc port> <out.csv> [seconds]
"""
import os, select, struct, sys, termios, time
port, out = sys.argv[1], sys.argv[2]
secs = float(sys.argv[3]) if len(sys.argv) > 3 else 150
ALLOWED = {101, 104, 105, 110}
fd = os.open(port, os.O_RDWR | os.O_NOCTTY)
a = termios.tcgetattr(fd); a[0]=a[1]=a[3]=0; a[2]=termios.CS8|termios.CREAD|termios.CLOCAL; a[4]=a[5]=termios.B115200
a[6][termios.VMIN], a[6][termios.VTIME] = 0, 1
termios.tcsetattr(fd, termios.TCSANOW, a); termios.tcflush(fd, termios.TCIOFLUSH)
def msp(cmd):
    assert cmd in ALLOWED
    os.write(fd, b"$M<" + bytes([0, cmd, cmd])); buf, end = b"", time.time() + 0.12
    while time.time() < end:
        if select.select([fd], [], [], 0.02)[0]: buf += os.read(fd, 512)
        i = buf.find(b"$M>")
        if i >= 0 and len(buf) >= i + 6 and len(buf) >= i + 6 + buf[i + 3]: return buf[i + 5:i + 5 + buf[i + 3]]
    return None
f = open(out, "w"); f.write("t,flags,m1,m2,m3,m4,roll,pitch,yaw,thr,aux1,aux2,rssi,vbat_dv\n"); f.flush()
t0 = time.time(); last_an = 0; rssi = ""; vb = ""
while time.time() - t0 < secs:
    t = time.time() - t0
    st = msp(101); mo = msp(104); rc = msp(105)
    if time.time() - last_an > 0.4:
        an = msp(110)
        if an and len(an) >= 5: vb = an[0]; rssi = struct.unpack_from("<H", an, 3)[0]
        last_an = time.time()
    flags = struct.unpack_from("<I", st, 6)[0] if st and len(st) >= 10 else ""
    m = list(struct.unpack_from("<4H", mo, 0)) if mo and len(mo) >= 8 else ["", "", "", ""]
    r = list(struct.unpack_from("<6H", rc, 0)) if rc and len(rc) >= 12 else [""] * 6
    f.write(f"{t:.2f},{flags},{m[0]},{m[1]},{m[2]},{m[3]}," + ",".join(map(str, r)) + f",{rssi},{vb}\n")
    if int(t * 25) % 25 == 0: f.flush()
    time.sleep(0.01)
f.close(); print("recording finished")
