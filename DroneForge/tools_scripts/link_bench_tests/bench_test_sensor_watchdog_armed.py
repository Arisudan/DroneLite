#!/usr/bin/env python3
"""Live test of the PC-side sensor watchdog (port of Nimbus OptrangeFailsafe). PROPS OFF, drone secured.
Arms, raises throttle to 5.5% (just above the 5% engage threshold) for a short window, with a motor-command guard and logs what the watchdog does and what the FC sees.
Ends after a hard cap, or 3 s after the watchdog trips, then disarms. FC port is read with read-only MSP (101/104/105).
usage: bench_test_sensor_watchdog_armed.py <fc_port> <adapter_port> [label]
"""
import hashlib, math, os, select, struct, sys, termios, time
sys.path.insert(0, "/home/htic/Music/DroneForge/software/pc")
from nimbus_pc.sender import Controls
from nimbus_pc.session import Session
from nimbus_pc.watchdog import NAMES

FC, ADP = sys.argv[1], sys.argv[2]
LABEL = sys.argv[3] if len(sys.argv) > 3 else "run"
ARM_AT, THR_AT, WINDOW = 8.0, 11.0, 3.0      # seconds: arm switch, throttle up, how long throttle stays engaged
CAP = THR_AT + WINDOW + 3.0
THR = 0.055
MOTOR_ABORT = 1400                            # commanded-motor guard: disarm at once above this (bench I-term windup)

fd = os.open(FC, os.O_RDWR | os.O_NOCTTY)
a = termios.tcgetattr(fd); a[0] = a[1] = a[3] = 0; a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL; a[4] = a[5] = termios.B115200
a[6][termios.VMIN], a[6][termios.VTIME] = 0, 1
termios.tcsetattr(fd, termios.TCSANOW, a); termios.tcflush(fd, termios.TCIOFLUSH)


def msp(cmd):
    assert cmd in (101, 104, 105)
    os.write(fd, b"$M<" + bytes([0, cmd, cmd])); buf, end = b"", time.time() + 0.15
    while time.time() < end:
        if select.select([fd], [], [], 0.02)[0]: buf += os.read(fd, 512)
        i = buf.find(b"$M>")
        if i >= 0 and len(buf) >= i + 6 and len(buf) >= i + 6 + buf[i + 3]: return buf[i + 5:i + 5 + buf[i + 3]]


payloads = []          # (t, md5) of every 0xD3 payload received
last_opt = {}


kinds = {}      # kind -> [t of every frame]
linkseries = []  # (t, up_lq, down_lq, up_rssi, down_rssi, snr)


def on_tel(name, values):
    kinds.setdefault(name, []).append(time.monotonic() - t0)
    if name == "link":
        linkseries.append((time.monotonic() - t0, values['up_lq'], values['down_lq'], values['up_rssi1_dbm'], values['down_rssi_dbm'], values['up_snr_db']))
    if name == "optrange":
        last_opt.update(values)
        payloads.append((time.monotonic() - t0, hashlib.md5(repr(sorted(values.items())).encode()).hexdigest()[:6]))


if msp(101) and struct.unpack_from("<I", msp(101), 6)[0] & 1:
    sys.exit("FC is armed: disarm first")
t0 = time.monotonic()
s = Session(ADP, hz=100, allow_arm=True, ping=True, watch_optrange=True, on_telemetry=on_tel)
s.start()
tripped_at = None; engaged_at = None; motors_now = None
print(f"[{LABEL}] arm at {ARM_AT}s, throttle {THR:.1%} at {THR_AT}s for {WINDOW}s, motor guard {MOTOR_ABORT}")
aborted = False
try:
    while True:
        t = time.monotonic() - t0
        arm = t >= ARM_AT
        thr = THR if THR_AT <= t < THR_AT + WINDOW and not (tripped_at is not None and t > tripped_at + 1.0) else 0.0
        s.set_controls(Controls(arm=arm, throttle=thr))
        wd = s.watchdog
        if wd.engaged and engaged_at is None: engaged_at = t
        if wd.forcing_zero and tripped_at is None: tripped_at = t
        if int(t * 2.5) != int((t - 0.05) * 2.5):          # ~every 0.4 s
            st = msp(101); mo = msp(104); rc = msp(105)
            armed = bool(struct.unpack_from("<I", st, 6)[0] & 1) if st else None
            motors = struct.unpack("<4H", mo[:8]) if mo else None
            motors_now = motors
            thr_ch = struct.unpack(f"<{len(rc)//2}H", rc)[3] if rc else None
            print(f"t={t:5.1f}s wd={NAMES[wd.state]:16s} engaged={wd.engaged!s:5} FCarmed={armed} FCthrottle={thr_ch} motors={motors} "
                  f"dist={last_opt.get('distance_m')} flowQ={last_opt.get('flow_quality')} flow=({last_opt.get('flow_x_raw')},{last_opt.get('flow_y_raw')})", flush=True)
        if motors_now and max(motors_now) > MOTOR_ABORT:
            aborted = True; print(f'!! motor guard tripped ({max(motors_now)}): disarming'); break
        if t > CAP:
            break
        time.sleep(0.01)
finally:
    s.set_controls(Controls())          # disarm, throttle minimum
    time.sleep(1.0)
    st = msp(101)
    print("after test, FC armed:", bool(struct.unpack_from('<I', st, 6)[0] & 1) if st else None)
    s.stop()
win = [h for (tt, h) in payloads if engaged_at is not None and tt >= engaged_at]
print(f"\nRESULT [{LABEL}]: engaged at {engaged_at}, watchdog tripped at {tripped_at}"
      + (f" (+{tripped_at - engaged_at:.1f}s after engage), state {NAMES[s.watchdog.state]}" if tripped_at and engaged_at else " (never tripped)"))
print("motor guard aborted the run" if aborted else "motor guard not needed")
print(f"0xD3 payloads after engage: {len(win)} frames, {len(set(win))} distinct")

print("\nframe gaps > 0.5 s after engage, per telemetry kind (a link dropout hits every kind, an FC/sensor stall only some):")
for k, v in sorted(kinds.items()):
    v = [x for x in v if engaged_at and engaged_at - 1 <= x]
    g = [(a, b - a) for a, b in zip(v, v[1:]) if b - a > 0.5]
    print(f"  {k:12s} {len(v):3d} frames, gaps>0.5s: " + (", ".join(f"{d:.1f}s@t={a:.1f}" for a, d in g) or "none"))

print("\nlink series (t, uplink LQ, downlink LQ, uplink RSSI, downlink RSSI, SNR) around engage:")
last = -9
for (tt, ul, dl, ur, dr, sn) in linkseries:
    if engaged_at and engaged_at - 3 <= tt <= engaged_at + 7 and tt - last >= 0.5:
        last = tt; print(f"  t={tt:5.1f} upLQ {ul:3d} downLQ {dl:3d} upRSSI {ur:4d} downRSSI {dr:4d} SNR {sn}")
