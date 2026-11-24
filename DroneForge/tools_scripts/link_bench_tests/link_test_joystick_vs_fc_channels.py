#!/usr/bin/env python3
"""End-to-end link test: Boxer USB joystick (HID) vs channels the FC receives (MSP_RC, read-only).
usage: link_test_joystick_vs_fc_channels.py <joystick dev> <fc port> <out_prefix>
Sends only MSP read requests 105 (RC) and 110 (ANALOG) to the FC.
"""
import fcntl, os, select, statistics as st, struct, subprocess, sys, termios, time

js_dev, fc_port, prefix = sys.argv[1:4]
READ_ONLY = {105, 110}

# joystick
jfd = os.open(js_dev, os.O_RDONLY | os.O_NONBLOCK)
n_axes = struct.unpack("B", fcntl.ioctl(jfd, 0x80016A11, b"\0"))[0]
n_btn = struct.unpack("B", fcntl.ioctl(jfd, 0x80016A12, b"\0"))[0]
name = fcntl.ioctl(jfd, 0x80006A13 + (128 << 16), b"\0" * 128).split(b"\0")[0].decode(errors="replace")
print(f"joystick: {name} | axes={n_axes} buttons={n_btn}")

# FC
ffd = os.open(fc_port, os.O_RDWR | os.O_NOCTTY)
a = termios.tcgetattr(ffd); a[0] = a[1] = a[3] = 0; a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL; a[4] = a[5] = termios.B115200
a[6][termios.VMIN], a[6][termios.VTIME] = 0, 1
termios.tcsetattr(ffd, termios.TCSANOW, a); termios.tcflush(ffd, termios.TCIOFLUSH)


def msp(cmd, timeout=0.15):
    assert cmd in READ_ONLY
    os.write(ffd, b"$M<" + bytes([0, cmd, cmd]))
    buf, end = b"", time.time() + timeout
    while time.time() < end:
        if select.select([ffd], [], [], 0.02)[0]:
            buf += os.read(ffd, 512)
        i = buf.find(b"$M>")
        if i >= 0 and len(buf) >= i + 6 and len(buf) >= i + 6 + buf[i + 3]:
            return buf[i + 5:i + 5 + buf[i + 3]]
    return None


def say(t):
    subprocess.Popen(["pw-play", "/usr/share/sounds/freedesktop/stereo/bell.oga"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.Popen(["spd-say", "-w", t], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.Popen(["notify-send", "-u", "critical", "-t", "5000", "Link test", t], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


phases = [("COUNTDOWN", 6, "Link test starts in five seconds. Hands off the sticks."),
          ("IDLE", 5, "Hands off the sticks."),
          ("LEFT_UD", 7, "Left stick: move up and down slowly."),
          ("LEFT_LR", 7, "Left stick: move left and right."),
          ("RIGHT_UD", 7, "Right stick: move up and down."),
          ("RIGHT_LR", 7, "Right stick: move left and right."),
          ("SWITCHES", 14, "Flip every switch one at a time, then put them back.")]
axes = [0] * 32
rows = []  # (t, phase, axes snapshot, rc list, rssi)
rssi = None
last_an = 0
for ph, dur, text in phases:
    say(text)
    end = time.time() + dur
    while time.time() < end:
        while select.select([jfd], [], [], 0)[0]:
            try:
                d = os.read(jfd, 8)
            except BlockingIOError:
                break
            if len(d) < 8:
                break
            _, val, typ, num = struct.unpack("IhBB", d)
            if (typ & 0x7F) == 2 and num < 32:
                axes[num] = val
        rc = msp(105)
        rcv = list(struct.unpack(f"<{len(rc)//2}H", rc)) if rc else None
        if time.time() - last_an > 0.5:
            an = msp(110)
            if an and len(an) >= 7:
                rssi = struct.unpack_from("<H", an, 3)[0]
            last_an = time.time()
        rows.append((time.time(), ph, list(axes[:max(n_axes, 1)]), rcv, rssi))
        time.sleep(0.03)
say("Test finished.")

good = [r for r in rows if r[3]]
print(f"samples: {len(rows)} | with FC channel data: {len(good)} | FC channel count: {len(good[0][3]) if good else 0}")
if not good:
    print("NO CHANNEL DATA from the FC: receiver not linked, or not on the RX serial port, or wrong protocol")
    sys.exit(0)
print("FC link quality (RSSI field from MSP_ANALOG, 0-1023): min/max =", min(r[4] for r in rows if r[4] is not None), max(r[4] for r in rows if r[4] is not None))
nch = len(good[0][3])


def corr(x, y):
    mx, my = st.mean(x), st.mean(y)
    sx = sum((a - mx) ** 2 for a in x) ** 0.5
    sy = sum((b - my) ** 2 for b in y) ** 0.5
    return 0.0 if sx == 0 or sy == 0 else sum((a - mx) * (b - my) for a, b in zip(x, y)) / (sx * sy)


print(f"\n{'FC ch':>5s} {'FC min..max (us)':>18s} | best Boxer axis | corr  | axis min..max (raw)")
for c in range(min(nch, 12)):
    fv = [r[3][c] for r in good]
    if max(fv) - min(fv) < 5:
        print(f"{c+1:5d} {min(fv):8d}..{max(fv):<8d} | (no movement)")
        continue
    best = max(range(len(good[0][2])), key=lambda k: abs(corr(fv, [r[2][k] for r in good])))
    av = [r[2][best] for r in good]
    print(f"{c+1:5d} {min(fv):8d}..{max(fv):<8d} | axis {best+1:<10d} | {corr(fv, av):+.3f} | {min(av)}..{max(av)}")
print("\nper-phase FC channel movement (channels 1-8, range in us):")
for ph, _, _ in phases:
    r = [x for x in good if x[1] == ph]
    if not r:
        continue
    parts = []
    for c in range(min(nch, 8)):
        v = [x[3][c] for x in r]
        parts.append(f"ch{c+1}:{max(v)-min(v):4d}")
    print(f"  {ph:10s} " + "  ".join(parts))
