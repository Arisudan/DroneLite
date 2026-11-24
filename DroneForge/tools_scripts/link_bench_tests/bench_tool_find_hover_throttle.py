#!/usr/bin/env python3
"""Find the real lift-off / hover throttle. PROPS ON, DRONE TETHERED (short slack, clear area), a person on the SAFE key and a hand near the battery plug.

All throttle values are PERCENT of the throttle stick (0 = minimum, 100 = full).
  bench_tool_find_hover_throttle.py --adapter <CP2102> --fc <FC usb> --start-pct 10 --cap-pct 40 --rate-pct 1.5 --apply
  bench_tool_find_hover_throttle.py --adapter <CP2102> --fc <FC usb> --start-pct 50 --cap-pct 50 --hold-s 4      (just hold 50 % for 4 s and see if it lifts)
  bench_tool_find_hover_throttle.py --sim                                                                          (dry run in the simulator, no hardware)
Arms at zero throttle (keep the drone STILL for the first ~5 s: gyro calibration), then raises the throttle from start to cap at rate %/s,
holds the cap for --hold-s, and stops (disarms) at: lift-off (range +0.12 m), cap + hold done, tilt > 25 deg, link/data loss, 60 s, Enter/Ctrl-C.
It prints the FC's real MOTOR outputs each second (read-only, over the radio link; or over USB with --fc): that tells "throttle not reaching the motors" from "motors too weak".
Every run saves a log to docs/hil/ramp_<time>.json (sent throttle, range, tilt, motor outputs) for review."""
import argparse, json, select, sys, time
sys.path.insert(0, "/home/htic/Music/DroneForge/software/pc")
from nimbus_core.core import NimbusCore
from nimbus_core.throttle_ramp import RampConfig
from nimbus_pc.sender import Controls

LOG_DIR = "/home/htic/Music/DroneForge/docs/hil"
NAMES = ["NOGYRO","FAILSAFE","RXLOSS","BADRX","BOXFAILSAFE","RUNAWAY","CRASH","THROTTLE","ANGLE","BOOTGRACE","NOPREARM","LOAD","CALIB","CLI","CMS","BST","MSP","PARALYZE","GPS","RESCUE_SW","RPMFILTER","REBOOT_REQD","DSHOT_BBANG","NO_ACC_CAL","MOTOR_PROTO","ARMSWITCH"]

ap = argparse.ArgumentParser()
ap.add_argument("--adapter"); ap.add_argument("--fc", default=None, help="OPTIONAL FC USB port (read-only). Without it the motor outputs and arming blockers are read over the RADIO link (MSP-over-CRSF), no cable")
ap.add_argument("--sim", action="store_true")
ap.add_argument("--start-pct", type=float, default=10.0); ap.add_argument("--cap-pct", type=float, default=35.0)
ap.add_argument("--rate-pct", type=float, default=1.5, help="percent of throttle per second")
ap.add_argument("--hold-s", type=float, default=3.0, help="hold the cap this long before giving up")
ap.add_argument("--lift-m", type=float, default=0.12, help="range rise that counts as lift-off")
ap.add_argument("--auto-level", action="store_true", help="if the drone rests more than 0.15 deg from level: send the accelerometer calibration gesture over the radio (DISARMED, motors do not spin) and re-check before arming")
ap.add_argument("--allow-tilted", action="store_true", help="arm even if the drone rests more than 0.15 deg from level (lift-off is then unlikely: the FC winds up against the tilt)")
ap.add_argument("--apply", action="store_true", help="on a lift result, offer to save the hover throttle estimate to the flight settings")
a = ap.parse_args()
if not a.sim and not a.adapter:
    sys.exit("give --adapter <port> or --sim")
if not (5 <= a.start_pct <= 85) or not (10 <= a.cap_pct <= 95) or a.start_pct > a.cap_pct or not (0.2 <= a.rate_pct <= 20):
    sys.exit("limits: start 5..85 %, cap 10..95 % (and >= start), rate 0.2..20 %/s")
if not a.sim:
    print(f"*** THIS ARMS THE DRONE AND SPINS THE MOTORS: throttle {a.start_pct:.0f}% -> {a.cap_pct:.0f}% at {a.rate_pct:g}%/s, hold {a.hold_s:g}s. PROPS ON, drone TETHERED with short slack, clear area, someone on SAFE, hand near the battery plug. ***")
    for i in (3, 2, 1):
        print(f"starting in {i} ... (Ctrl-C aborts)", flush=True); time.sleep(1)
fc = None
if a.fc and not a.sim:
    from nimbus_pc.hover.hil import FcReader
    fc = FcReader(a.fc)
core = NimbusCore()
if a.sim: core.connect_sim()
else: core.connect(a.adapter)
core.set_source("external"); core.set_allow_arm(True)
_rt = {"v": None}; _lv = {"done": False}
def _rest(m):
    if m[0] == "attitude":
        import math as _m
        _rt["v"] = (_m.degrees(m[1]["pitch_rad"]), _m.degrees(m[1]["roll_rad"]))
wl = {"motors": None, "thr_us": None, "t": 0.0, "diag": {}}                                  # the FC's motor outputs, sent wirelessly in its flight-mode text
def _on_tel(m):
    if m[0] == "flight_mode":
        d = m[1]
        if "motors" in d:
            wl.update(motors=d["motors"], thr_us=d["thr_us"], t=time.monotonic())
        for k in ("pid_p", "pid_i", "pid_d", "gyro_dps10", "pid_sum"):
            if k in d: wl["diag"][k] = d[k]
from nimbus_core import topics as _T
core.bus.subscribe(_T.TELEMETRY, _on_tel); core.bus.subscribe(_T.TELEMETRY, _rest)
probe = None
if not fc and not a.sim:
    from nimbus_core.radio_fc import RadioFcProbe
    probe = RadioFcProbe(core.sess, period_s=0.5); probe.start()
core.ramp.cfg = RampConfig(start_thr=a.start_pct / 100, rate_per_s=a.rate_pct / 100, cap_thr=a.cap_pct / 100, hold_at_cap_s=a.hold_s, lift_height_m=a.lift_m)
t0 = time.monotonic(); started = False; last = 0.0; last_pf = 0.0; good = 0; motor_log = []
try:
    while time.monotonic() - t0 < 120:
        core.tick(); time.sleep(0.02)
        if select.select([sys.stdin], [], [], 0)[0]:
            line = sys.stdin.readline()
            if line:                                          # an Enter key press ('\n'); an empty string is a closed stdin, not an abort
                print("operator abort"); core.ramp.stop("operator"); break
        if not started and time.monotonic() - t0 > (3.0 if a.sim else 6.0) and time.monotonic() - last_pf > 1.0:
            last_pf = time.monotonic()
            if _rt["v"] and not started and good == 0:
                big = max(abs(_rt["v"][0]), abs(_rt["v"][1])) > 0.15
                print(f"rest tilt: pitch {_rt['v'][0]:+.2f} deg, roll {_rt['v'][1]:+.2f} deg" + ("  <- NOT LEVEL (over 0.15 deg)" if big else "  (level)"), flush=True)
                if big and not a.allow_tilted and a.auto_level and not _lv["done"] and core.autonomy.fc_mode.endswith("*"):
                    _lv["done"] = True
                    print("auto-level: sending the accelerometer calibration gesture (DISARMED, arm switch low, motors do not spin). Keep the drone still ...", flush=True)
                    t1 = time.monotonic()
                    while time.monotonic() - t1 < 2.5:
                        core.set_external_controls(Controls(roll=0.0, pitch=-1.0, yaw=-1.0, throttle=1.0, arm=False)); core.tick(); time.sleep(0.02)
                    t1 = time.monotonic()
                    while time.monotonic() - t1 < 5.0:
                        core.set_external_controls(Controls()); core.tick(); time.sleep(0.02)
                    _rt["v"] = None; last_pf = 0.0
                    continue
                if big and not a.allow_tilted:
                    core.safe_now(); time.sleep(0.3)
                    print("NOT ARMING: the drone rests more than 0.15 deg from level, lift-off is unlikely (every run today with 0.2 deg or more did not lift; every run within 0.1 deg did).\n"
                          "Re-level the FC at this exact spot (fc_calibrate_accel_level_usb.py, battery off, FC on USB) and retry, or add --allow-tilted to arm anyway.")
                    sys.exit(2)
            ok, why = core.autonomy.preflight(for_ramp=True)
            good = good + 1 if ok else 0
            if not ok:
                print("waiting for preflight:", "; ".join(why), flush=True)
                if time.monotonic() - t0 > 36.0: sys.exit("preflight still failing after 30 s: " + "; ".join(why))
            elif good >= 2:
                started = core.ramp.start()
        if started and time.monotonic() - last > 1.0:
            last = time.monotonic(); r = core.ramp
            m = fc.motors() if fc else (probe.motors if (probe and probe.t_motors and time.monotonic() - probe.t_motors < 2.0) else None)
            if not m and wl["motors"] and time.monotonic() - wl["t"] < 2.0:
                m = wl["motors"] + [f"thr {wl['thr_us']}"]
            if m: motor_log.append((round(time.monotonic() - t0, 1), r.thr, m))
            dg = wl["diag"]
            extra = (f"  I[r,p,y]={dg['pid_i']} P={dg.get('pid_p')} D={dg.get('pid_d')}" if "pid_i" in dg else "") + (f"  gyro(0.1deg/s)={dg['gyro_dps10']} sum={dg['pid_sum']}" if "gyro_dps10" in dg else "")
            print(f"{r.phase:8s} throttle {r.thr * 100:5.1f}%  range {r._range if r._range is not None else float('nan'):.3f} m  tilt {max(abs(r._pitch or 0), abs(r._roll or 0)):.1f}"
                  + (f"  motors {m}" if m else ("  motors n/a (no motor data from the FC yet)" if probe else "")), flush=True)
        if started and core.ramp.phase == "done": break
except KeyboardInterrupt:
    core.ramp.stop("operator")
finally:
    core.safe_now(); time.sleep(0.4)
    blockers = None
    if probe:
        probe.stop(); blockers = probe.blockers
    if fc:
        try:
            b = fc.blockers(); blockers = [NAMES[i] for i in range(len(NAMES)) if (b or 0) >> i & 1]
        except Exception:
            pass
        fc.close()
    core.disconnect()
res = core.ramp.result or {"reason": "aborted"}
for k in ("lift_throttle", "hover_throttle_est", "last_throttle"):
    if res.get(k) is not None: res[k + "_pct"] = round(res[k] * 100, 1)
print(json.dumps(res, indent=1))
if res.get("reason") == "limit":
    print(f"No lift-off up to {a.cap_pct:.0f}% (held {a.hold_s:g} s). Check the motor outputs above: if they reached ~{1000 + a.cap_pct * 10:.0f} the throttle is reaching the motors and the thrust is too low "
          "for the tether/weight; if they stayed near 1050 the throttle is not getting through (check the FC's throttle/motor limits).")
if res.get("reason") == "did_not_arm":
    print("FC did not arm. Keep the drone completely STILL on the ground for the first 5 s (gyro calibration), then retry.")
    if blockers is not None: print("FC arming blockers now:", blockers or "none")
import os, datetime
os.makedirs(LOG_DIR, exist_ok=True)
path = f"{LOG_DIR}/ramp_{datetime.datetime.now():%Y%m%d_%H%M%S}.json"
json.dump({"args": vars(a), "result": res, "log_t_thr_range_pitch_roll": core.ramp.log, "motors_t_thr_outputs": motor_log, "fc_blockers_after": blockers}, open(path, "w"))
print("log saved:", path)
if a.apply and res.get("reason") == "lift" and not a.sim:
    from nimbus_core import settings as S
    est = round(res["hover_throttle_est"], 3)
    print(f"lift-off at {res['lift_throttle'] * 100:.1f}%; hover throttle estimate {est * 100:.1f}%. Repeat the ramp 2-3 times and compare before trusting one run.")
    if input(f"save hover throttle {est * 100:.1f}% as MEASURED? type yes: ").strip().lower() == "yes":
        cur = S.load(); cur.thr_hover0, cur.measured = est, True; cur.thr_max = round(min(max(est * 1.2, est + 0.10), 1.0), 2)
        if res.get('batt_v'): cur.batt_ref_v = res['batt_v']          # the pack voltage this lift-off was measured at
        S.save(cur); print("saved to", S.PATH)
