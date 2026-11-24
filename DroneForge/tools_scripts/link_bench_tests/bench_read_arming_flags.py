"""Read Betaflight arming-disabled flags over time with the sender running (read-only MSP). usage: <fc_port> <adapter_port>"""
import os, select, struct, subprocess, sys, termios, time, signal
FC, ADP = sys.argv[1], sys.argv[2]
fd = os.open(FC, os.O_RDWR | os.O_NOCTTY)
a = termios.tcgetattr(fd); a[0] = a[1] = a[3] = 0; a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL; a[4] = a[5] = termios.B115200
a[6][termios.VMIN], a[6][termios.VTIME] = 0, 1
termios.tcsetattr(fd, termios.TCSANOW, a); termios.tcflush(fd, termios.TCIOFLUSH)
def msp(cmd):
    assert cmd in (150, 105)
    os.write(fd, b"$M<" + bytes([0, cmd, cmd])); buf, end = b"", time.time() + 0.3
    while time.time() < end:
        if select.select([fd], [], [], 0.02)[0]: buf += os.read(fd, 512)
        i = buf.find(b"$M>")
        if i >= 0 and len(buf) >= i + 6 and len(buf) >= i + 6 + buf[i + 3]: return buf[i + 5:i + 5 + buf[i + 3]]
NAMES = ["NOGYRO","FAILSAFE","RXLOSS","BADRX","BOXFAILSAFE","RUNAWAY","CRASH","THROTTLE","ANGLE","BOOTGRACE","NOPREARM","LOAD","CALIB","CLI","CMS","BST","MSP","PARALYZE","GPS","RESCUE_SW","RPMFILTER","REBOOT_REQD","DSHOT_BBANG","NO_ACC_CAL","MOTOR_PROTO","ARMSWITCH"]
def flags():
    p = msp(150)
    if not p or len(p) < 22: return None, p
    n = p[15]; off = 16 + n   # byteCount at 15 per BF4.5 STATUS_EX layout (11 + load/gyro 4 + flagbytes)
    return None, p
def show(tag):
    p = msp(150)
    print(tag, "raw len", len(p) if p else None, p.hex() if p else "")
    if p and len(p) >= 22:
        cnt = p[15]; # number of flight mode flag bytes
        idx = 16 + cnt
        na = p[idx]; af = struct.unpack_from("<I", p, idx + 1)[0]
        print("   arming-disable mask 0x%X ->" % af, [NAMES[i] for i in range(len(NAMES)) if af >> i & 1] or "none")
pr = subprocess.Popen(["python3", "-m", "nimbus_pc", "send", "--port", ADP, "--demo", "idle", "--duration", "60"],
                      cwd="/home/htic/Music/DroneForge/software/pc", stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
t0 = time.time()
while time.time() - t0 < 40:
    show("t=%4.1fs sender on, arm LOW:" % (time.time() - t0)); time.sleep(4)
pr.send_signal(signal.SIGKILL)
