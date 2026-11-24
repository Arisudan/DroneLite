"""ARMS THE FC (props OFF!): arms with the sender running, hard-kills the sender, logs the failsafe disarm. usage: <fc_port> <adapter_port>"""
import os, select, struct, subprocess, sys, termios, time, signal
FC, ADP = sys.argv[1], sys.argv[2]
fd = os.open(FC, os.O_RDWR | os.O_NOCTTY)
a = termios.tcgetattr(fd); a[0] = a[1] = a[3] = 0; a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL; a[4] = a[5] = termios.B115200
a[6][termios.VMIN], a[6][termios.VTIME] = 0, 1
termios.tcsetattr(fd, termios.TCSANOW, a); termios.tcflush(fd, termios.TCIOFLUSH)
READ_ONLY = (101, 104, 105)
def msp(cmd):
    assert cmd in READ_ONLY
    os.write(fd, b"$M<" + bytes([0, cmd, cmd])); buf, end = b"", time.time() + 0.15
    while time.time() < end:
        if select.select([fd], [], [], 0.02)[0]: buf += os.read(fd, 512)
        i = buf.find(b"$M>")
        if i >= 0 and len(buf) >= i + 6 and len(buf) >= i + 6 + buf[i + 3]: return buf[i + 5:i + 5 + buf[i + 3]]
def snap():
    st, mo, rc = msp(101), msp(104), msp(105)
    armed = (struct.unpack_from("<I", st, 6)[0] & 1) if st and len(st) >= 10 else None
    m = struct.unpack("<4H", mo[:8]) if mo and len(mo) >= 8 else None
    r = struct.unpack(f"<{len(rc)//2}H", rc) if rc else None
    return armed, m, (r[3] if r and len(r) > 3 else None), (r[4] if r and len(r) > 4 else None)
print("baseline (no sender):", snap())
p = subprocess.Popen(["python3", "-m", "nimbus_pc", "send", "--port", ADP, "--demo", "idle", "--arm", "--arm-after", "10", "--allow-arm", "--duration", "120"],
                     cwd="/home/htic/Music/DroneForge/software/pc", stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
t0 = time.time(); killed = None
while time.time() - t0 < 50:
    t = time.time() - t0
    if killed is None and t >= 22:
        p.send_signal(signal.SIGKILL); killed = t; print(f"--- t={t:.1f}s: SENDER KILLED (hard link loss) ---")
    armed, m, thr, aux1 = snap()
    print(f"t={t:5.1f}s armed={armed} motors={m} thr_ch={thr} arm_ch={aux1}", flush=True)
    time.sleep(0.9)
if p.poll() is None: p.kill()
