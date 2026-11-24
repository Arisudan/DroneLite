#!/usr/bin/env python3
"""Boxer arming-guard test over the real link. PROPS OFF, drone secured, THROTTLE STICK AT THE BOTTOM unless a step says otherwise.
Logs every change of: FC armed state, PC arm_blocked flag, Boxer arm switch, throttle. Runs for a fixed time (default 75 s).
Safety: if any FC motor command exceeds 1300 the PC goes SAFE (disarm) and the run ends. Closing the pad also ends in SAFE.
usage: bench_test_gamepad_arm_guards.py <js_dev> <fc_port> <adapter_port> [seconds]
"""
import os, select, struct, subprocess, sys, termios, time
sys.path.insert(0, "/home/htic/Music/DroneForge/software/pc")
from nimbus_pc.gamepad import GamepadInput
from nimbus_pc.sender import Controls
from nimbus_pc.session import Session

JS, FC, ADP = sys.argv[1:4]
SECONDS = float(sys.argv[4]) if len(sys.argv) > 4 else 75.0
fd = os.open(FC, os.O_RDWR | os.O_NOCTTY)
a = termios.tcgetattr(fd); a[0] = a[1] = a[3] = 0; a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL; a[4] = a[5] = termios.B115200
a[6][termios.VMIN], a[6][termios.VTIME] = 0, 1
termios.tcsetattr(fd, termios.TCSANOW, a); termios.tcflush(fd, termios.TCIOFLUSH)


def msp(cmd):
    assert cmd in (101, 104, 105, 150)
    os.write(fd, b"$M<" + bytes([0, cmd, cmd])); buf, end = b"", time.time() + 0.15
    while time.time() < end:
        if select.select([fd], [], [], 0.02)[0]: buf += os.read(fd, 512)
        i = buf.find(b"$M>")
        if i >= 0 and len(buf) >= i + 6 and len(buf) >= i + 6 + buf[i + 3]: return buf[i + 5:i + 5 + buf[i + 3]]


FLAGS = ["NOGYRO","FAILSAFE","RXLOSS","BADRX","BOXFAILSAFE","RUNAWAY","CRASH","THROTTLE","ANGLE","BOOTGRACE","NOPREARM","LOAD","CALIB","CLI","CMS","BST","MSP","PARALYZE","GPS","RESCUE_SW","RPMFILTER","REBOOT_REQD","DSHOT_BBANG","NO_ACC_CAL","MOTOR_PROTO","ARMSWITCH"]


def blockers():
    p = msp(150)
    if not p or len(p) < 22: return None
    idx = 16 + p[15]
    mask = struct.unpack_from("<I", p, idx + 1)[0]
    return [FLAGS[i] for i in range(len(FLAGS)) if mask >> i & 1]


pad = GamepadInput(JS)
s = Session(ADP, hz=100, allow_arm=True, ping=True)
s.start()
t0 = time.monotonic(); last = None; events = []; guard = False
try:
    while time.monotonic() - t0 < SECONDS:
        pad.poll()
        c = pad.controls()
        s.set_controls(c)
        st, mo = msp(101), msp(104)
        armed = bool(struct.unpack_from("<I", st, 6)[0] & 1) if st else None
        motors = struct.unpack("<4H", mo[:8]) if mo else (0, 0, 0, 0)
        sw = pad._axis(pad.map.arm) > 0
        rcv = msp(105); rcv = struct.unpack(f"<{len(rcv)//2}H", rcv) if rcv else ()
        blk = tuple(blockers() or ()) if not armed else ()
        state = (armed, pad.arm_blocked, sw, c.throttle > 0.05, pad.connected, blk)
        if armed and not (last and last[0]):
            subprocess.Popen(['pw-play', '/usr/share/sounds/freedesktop/stereo/bell.oga'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            subprocess.Popen(['notify-send', '-u', 'critical', '-t', '6000', 'Arm test', 'FC ARMED (props off). Unplug the Boxer USB cable now if this is the unplug test.'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if state != last:
            t = time.monotonic() - t0
            events.append((t, state)); last = state
            print(f"t={t:5.1f}s  FC armed={armed!s:5}  PC arm_blocked={pad.arm_blocked!s:5}  Boxer arm switch={'ON ' if sw else 'OFF'}  "
                  f"throttle={'UP  ' if c.throttle > 0.05 else 'low '}  pad={pad.connected}  motors={motors}  FCch(thr,aux1-4)={rcv[3:8] if rcv else None}  FC blockers={list(blk)}", flush=True)
        if max(motors) > 1300:
            guard = True; print("!! motor guard: going SAFE"); break
        time.sleep(0.05)
finally:
    s.set_controls(Controls()); time.sleep(1.0)
    st = msp(101)
    print("end: FC armed =", bool(struct.unpack_from("<I", st, 6)[0] & 1) if st else None, "| motor guard used:", guard)
    s.stop(); pad.close()
