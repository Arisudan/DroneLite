#!/usr/bin/env python3
"""Why does the FC not arm? Sends arm HIGH at ZERO throttle for up to 8 s in ANGLE mode, then in ACRO mode, and prints the FC's arming
blockers, flight-mode text, armed flag and motor outputs twice a second. Disarms at the end of each trial.
PROPS OFF REQUIRED: a successful arm spins the motors at idle speed.
usage: bench_diag_why_fc_not_arming.py --adapter <CP2102 port> --fc <FC usb port>"""
import argparse, sys, time
sys.path.insert(0, "/home/htic/Music/DroneForge/software/pc")
from nimbus_pc.hover import hil
from nimbus_pc.sender import CRSF_MAX, CRSF_MIN, Controls
from nimbus_pc.session import Session

NAMES = ["NOGYRO","FAILSAFE","RXLOSS","BADRX","BOXFAILSAFE","RUNAWAY","CRASH","THROTTLE","ANGLE","BOOTGRACE","NOPREARM","LOAD","CALIB","CLI","CMS","BST","MSP","PARALYZE","GPS","RESCUE_SW","RPMFILTER","REBOOT_REQD","DSHOT_BBANG","NO_ACC_CAL","MOTOR_PROTO","ARMSWITCH"]
ap = argparse.ArgumentParser(); ap.add_argument("--adapter", required=True); ap.add_argument("--fc", required=True)
a = ap.parse_args()
if input("props must be OFF. type exactly  PROPS OFF  to continue: ").strip() != "PROPS OFF":
    sys.exit("not confirmed, nothing done")
last = {}; t0 = time.monotonic()
s = Session(a.adapter, hz=100.0, allow_arm=True, on_telemetry=lambda n, d: last.__setitem__(n, d))
f = hil.FcReader(a.fc); hil.MSP_READ_ONLY = (101, 104, 150)
s.start()
try:
    while time.monotonic() - t0 < 12:                     # let the link settle, disarmed
        s.set_controls(Controls()); time.sleep(0.01)
    print("link LQ", last.get("link", {}).get("up_lq"), "mode", last.get("flight_mode"))
    for label, aux2 in (("ANGLE mode (aux2 low)", CRSF_MIN), ("ACRO mode (aux2 high)", CRSF_MAX)):
        print(f"--- trial: {label}, arm HIGH, throttle 0 ---")
        t1 = time.monotonic(); armed_at = None
        while time.monotonic() - t1 < 8:
            s.set_controls(Controls(arm=True, aux2=aux2)); time.sleep(0.01)
            if int((time.monotonic() - t1) * 100) % 50 == 0:
                b = f.blockers(); ar = f.armed(); m = f.motors()
                print(f"  {time.monotonic()-t1:4.1f}s armed={ar} blockers={[NAMES[i] for i in range(len(NAMES)) if (b or 0) >> i & 1] or 'none'} mode={last.get('flight_mode',{}).get('mode')} motors={m}", flush=True)
                time.sleep(0.15)
                if ar and armed_at is None: armed_at = time.monotonic() - t1
            if armed_at is not None and time.monotonic() - t1 > armed_at + 1.0: break
        print("  RESULT:", f"armed after {armed_at:.1f} s" if armed_at is not None else "DID NOT ARM")
        t2 = time.monotonic()
        while time.monotonic() - t2 < 3:                  # disarm and let the FC settle (arm switch low)
            s.set_controls(Controls()); time.sleep(0.01)
finally:
    s.set_controls(Controls()); time.sleep(0.2); s.stop(); f.close()
