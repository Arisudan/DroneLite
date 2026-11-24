#!/usr/bin/env python3
"""Free-movement capture: Boxer USB joystick axes vs FC channels (MSP_RC read-only), saved to CSV.
usage: link_capture_joystick_vs_fc_channels_free.py <joystick dev> <fc port> <out.csv> [seconds]
"""
import fcntl, os, select, struct, subprocess, sys, termios, time
js_dev, fc_port, out = sys.argv[1:4]
secs = float(sys.argv[4]) if len(sys.argv) > 4 else 14
jfd = os.open(js_dev, os.O_RDONLY | os.O_NONBLOCK)
n_axes = struct.unpack("B", fcntl.ioctl(jfd, 0x80016A11, b"\0"))[0]
n_btn = struct.unpack("B", fcntl.ioctl(jfd, 0x80016A12, b"\0"))[0]
ffd = os.open(fc_port, os.O_RDWR | os.O_NOCTTY)
a = termios.tcgetattr(ffd); a[0]=a[1]=a[3]=0; a[2]=termios.CS8|termios.CREAD|termios.CLOCAL; a[4]=a[5]=termios.B115200
a[6][termios.VMIN], a[6][termios.VTIME] = 0, 1
termios.tcsetattr(ffd, termios.TCSANOW, a); termios.tcflush(ffd, termios.TCIOFLUSH)
def msp(cmd):
    assert cmd in (105, 110)
    os.write(ffd, b"$M<" + bytes([0, cmd, cmd])); buf, end = b"", time.time() + 0.15
    while time.time() < end:
        if select.select([ffd], [], [], 0.02)[0]: buf += os.read(ffd, 512)
        i = buf.find(b"$M>")
        if i >= 0 and len(buf) >= i + 6 and len(buf) >= i + 6 + buf[i + 3]: return buf[i + 5:i + 5 + buf[i + 3]]
    return None
subprocess.Popen(["pw-play", "/usr/share/sounds/freedesktop/stereo/bell.oga"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
subprocess.Popen(["spd-say", "-w", "Move all the sticks and flip all the switches now, for fourteen seconds."], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
subprocess.Popen(["notify-send", "-u", "critical", "-t", "6000", "Link test", "MOVE sticks and switches now"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
axes = [0]*32; btn = [0]*32; f = open(out, "w")
f.write("t," + ",".join(f"ax{i+1}" for i in range(n_axes)) + "," + ",".join(f"fc{i+1}" for i in range(16)) + ",rssi\n")
t0 = time.time(); rssi = ""; last_an = 0
while time.time() - t0 < secs:
    while select.select([jfd], [], [], 0)[0]:
        try: d = os.read(jfd, 8)
        except BlockingIOError: break
        if len(d) < 8: break
        _, v, typ, num = struct.unpack("IhBB", d)
        if (typ & 0x7F) == 2 and num < 32: axes[num] = v
    rc = msp(105)
    if time.time() - last_an > 0.5:
        an = msp(110)
        if an and len(an) >= 5: rssi = struct.unpack_from("<H", an, 3)[0]
        last_an = time.time()
    if rc:
        vals = list(struct.unpack(f"<{len(rc)//2}H", rc))[:16]
        f.write(f"{time.time()-t0:.2f}," + ",".join(str(x) for x in axes[:n_axes]) + "," + ",".join(str(x) for x in vals) + f",{rssi}\n")
    time.sleep(0.03)
f.close()
import csv, statistics as st
rows = list(csv.DictReader(open(out)))
print(f"joystick: axes={n_axes} buttons={n_btn} | rows={len(rows)}")
print("Boxer joystick axes (raw -32767..32767):")
for i in range(n_axes):
    v = [int(r[f'ax{i+1}']) for r in rows]; print(f"  axis {i+1}: {min(v):6d}..{max(v):6d}")
print("FC received channels (us):")
for i in range(16):
    v = [int(r[f'fc{i+1}']) for r in rows]
    print(f"  ch {i+1:2d}: {min(v):5d}..{max(v):5d}" + ("   <- moved" if max(v)-min(v) > 5 else ""))
r = [x['rssi'] for x in rows if x['rssi'] != '']
print("RSSI field min/max:", min(map(int, r)), max(map(int, r)))
