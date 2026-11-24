#!/usr/bin/env python3
"""Boxer (USB Joystick) -> PC sender -> TX -> RF -> RX -> FC: does every stick/switch reach the right FC channel? DISARMED.
The arm channel is forced low (allow_arm=False), so nothing can arm or spin. Move every stick to its full range and flip
every switch during the run. FC channels (MSP_RC, read-only) are in Betaflight order: roll, pitch, yaw, throttle, aux1..
usage: bench_check_gamepad_to_fc_channels.py <js_dev> <fc_port> <adapter_port> [seconds]
"""
import os, select, statistics as st, struct, sys, termios, time
sys.path.insert(0, "/home/htic/Music/DroneForge/software/pc")
from nimbus_pc.gamepad import GamepadInput
from nimbus_pc.sender import Controls
from nimbus_pc.session import Session

JS, FC, ADP = sys.argv[1:4]
SECONDS = float(sys.argv[4]) if len(sys.argv) > 4 else 30.0
EXPECT = {0: ("roll", 0), 1: ("pitch", 1), 2: ("yaw", 3), 3: ("throttle", 2), 5: ("aux2", 5), 6: ("aux3", 6)}   # FC idx -> (name, Boxer axis)

fd = os.open(FC, os.O_RDWR | os.O_NOCTTY)
a = termios.tcgetattr(fd); a[0] = a[1] = a[3] = 0; a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL; a[4] = a[5] = termios.B115200
a[6][termios.VMIN], a[6][termios.VTIME] = 0, 1
termios.tcsetattr(fd, termios.TCSANOW, a); termios.tcflush(fd, termios.TCIOFLUSH)


def rc():
    os.write(fd, b"$M<" + bytes([0, 105, 105])); buf, end = b"", time.time() + 0.15
    while time.time() < end:
        if select.select([fd], [], [], 0.02)[0]: buf += os.read(fd, 512)
        i = buf.find(b"$M>")
        if i >= 0 and len(buf) >= i + 6 and len(buf) >= i + 6 + buf[i + 3]:
            p = buf[i + 5:i + 5 + buf[i + 3]]; return struct.unpack(f"<{len(p)//2}H", p)


def corr(x, y, max_lag=8):
    """Pearson correlation, best over a small lag (the FC sees the stick ~100-200 ms later: RF + MSP sampling)."""
    best = 0.0
    for lag in range(0, max_lag + 1):
        a, b = (x[lag:], y[:len(y) - lag]) if lag else (x, y)
        if len(a) > 5:
            c = _corr0(a, b)
            best = c if abs(c) > abs(best) else best
    return best


def _corr0(x, y):
    mx, my = st.mean(x), st.mean(y)
    sx = sum((a - mx) ** 2 for a in x) ** 0.5; sy = sum((b - my) ** 2 for b in y) ** 0.5
    return 0.0 if sx == 0 or sy == 0 else sum((a - mx) * (b - my) for a, b in zip(x, y)) / (sx * sy)


pad = GamepadInput(JS)
s = Session(ADP, hz=100, allow_arm=False, ping=True)
s.start()
rows = []
t0 = time.monotonic()
try:
    while time.monotonic() - t0 < SECONDS:
        pad.poll()
        s.set_controls(pad.controls())
        r = rc()
        if r and len(r) >= 7 and time.monotonic() - t0 > 4:          # let the link settle first
            rows.append((r, list(pad._axes[:8])))
        time.sleep(0.05)
finally:
    s.stop(); pad.close()
print(f"samples: {len(rows)}   pad ready: {pad.ready}   FC armed-channel max: {max((r[0][4] for r in rows), default=None)} (must stay ~1000)")
ok = True
for fc_idx, (name, axis) in EXPECT.items():
    fv = [r[0][fc_idx] for r in rows]; ax = [r[1][axis] for r in rows]
    best = max(range(8), key=lambda k: abs(corr(fv, [r[1][k] for r in rows])))   # lag-tolerant
    c = corr(fv, ax); span = max(fv) - min(fv)
    good = abs(c) >= 0.9 and span >= 700 and best == axis
    ok &= good
    print(f"FC ch{fc_idx+1} {name:8s} {min(fv):4d}..{max(fv):4d} us (span {span:4d}) | Boxer axis {axis+1} corr {c:+.3f} | best axis {best+1} | {'PASS' if good else 'CHECK'}")
print("RESULT:", "all mapped channels follow the Boxer" if ok else "some channels did not follow (did you move every stick/switch through its full range?)")
