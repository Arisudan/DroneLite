#!/usr/bin/env python3
"""v4.7 ST bench check: DISARMED, PROPS OFF. This script never arms: it sends only disarmed, centred sticks (allow_arm=False).

Checks, from the FC's own telemetry (0xD6 state frame over ELRS), what the GCS / map / sim code ASSUME (sim_radio.py 'UNVERIFIED conventions', nimbus_core/kalman.py):
  0  firmware      the FC handles MSP2_DF_POS_AID (0x3F00) = v4.7 ST            (nimbus_gcs/fc_caps.py)
  1  pitch         nose DOWN            -> pitch_rad goes UP (+)
  2  roll          RIGHT side DOWN      -> roll_rad goes UP (+)
  3  yaw           turn CLOCKWISE (seen from above) -> yaw_rad goes UP (+) (heading, wraps at 2 pi)
  4  height        lift UP 0.3 m        -> range goes up, EKF vz > 0 (up +)
  5  flow forward  slide toward the NOSE -> flow velocity_forward > 0 and EKF forward speed > 0
  6  flow right    slide to the RIGHT    -> flow velocity_right > 0 and EKF right speed > 0
A bell + a desktop note + the terminal say what to do; each step: 4 s to get ready, 3 s of holding / moving, 3 s back to the start.
Hold the drone LEVEL, nose AWAY from you, 0.3-0.5 m over a textured floor (rug, wood, printed paper; not a plain white table).

usage: python3 tools_scripts/bench_v47.py --live   (self-paced live readout, recorded)
       python3 tools_scripts/bench_v47.py [--only roll,fwd,right,...] [adapter_port]   (step names: level pitch roll yaw up fwd right; level is always first)      (default: the CP2102 ELRS dongle; close the GCS first: it holds the dongle)
Writes docs/status_logs/<date>_bench-v47_signs.txt (+ .csv)."""
import glob, math, os, statistics as st, subprocess, sys, time

DF = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(DF, "software", "pc"))
from nimbus_pc.sender import Controls                     # noqa: E402
from nimbus_pc.session import Session                     # noqa: E402
from nimbus_pc.crsf import split_state                    # noqa: E402
from nimbus_gcs.fc_caps import AidSupport                 # noqa: E402

READY, ACT, BACK = 7.0, 4.0, 5.0      # 2026-11-23 first run: 4/3/3 s was too tight to follow
STEPS = [("level", "hold it LEVEL and still", None),
         ("pitch", "tip the NOSE DOWN about 20-30 deg and hold", "pitch"),
         ("roll", "tip the RIGHT side DOWN about 20-30 deg and hold", "roll"),
         ("yaw", "turn it CLOCKWISE (seen from above) about a quarter turn, slowly, keep it level", "yaw"),
         ("up", "lift it straight UP about 0.3 m, slowly, keep it level", "up"),
         ("fwd1", "slide it toward the NOSE (away from you) about 0.3 m, level, smooth", "fwd"),
         ("fwd2", "slide it toward the NOSE again (start from where you are now)", "fwd"),
         ("right1", "slide it to your RIGHT about 0.3 m, level, smooth", "right"),
         ("right2", "slide it to your RIGHT again", "right")]
t0 = time.monotonic(); rows = []


def now():
    return time.monotonic() - t0


def on_tel(name, d):
    if name == "state_compact":                              # the 0xD6 frame arrives whole from a bare Session: split it as the GCS core does
        for n2, d2 in split_state(d): on_tel(n2, d2)
        return
    if name == "kinematic":
        rows.append(("k", now(), d.get("roll_rad", 0.0), d.get("pitch_rad", 0.0), d.get("yaw_rad", 0.0), d.get("vx_mps", 0.0), d.get("vy_mps", 0.0), d.get("vz_mps", 0.0)))
    elif name == "optrange":
        rows.append(("o", now(), d.get("distance_m", 0.0), d.get("velocity_forward", 0.0), d.get("velocity_right", 0.0), d.get("flow_quality", 0), 0.0, 0.0))


def bell(msg):
    for cmd in (["pw-play", "/usr/share/sounds/freedesktop/stereo/bell.oga"], ["notify-send", "-u", "critical", "-t", "2500", "Bench check", msg]):
        try: subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except OSError: pass
    print(f"[{now():5.1f} s] {msg}", flush=True)


def win(kind, a, b):
    return [r for r in rows if r[0] == kind and a <= r[1] <= b]


def unwrap(v):
    if not v: return []
    out = [v[0]]
    for x in v[1:]:
        d = x - out[-1]; d -= 2 * math.pi * round(d / (2 * math.pi)); out.append(out[-1] + d)
    return out


def live(port: str, seconds: float = 90.0) -> int:
    """Self-paced: prints the FC's readings 5 times a second (no bells), records all of it. Never arms."""
    s = Session(port, hz=100.0, allow_arm=False, on_telemetry=on_tel); s.start(); last = 0.0
    print("LIVE readout (Ctrl+C to stop). Props off, disarmed. Hold the drone 0.4-0.5 m over a textured floor, nose AWAY from you.\n"
          "  1. Tip the RIGHT side down and hold:  watch 'roll'.\n  2. Slide SLOWLY to your RIGHT (keep it flat), stop, wait 2 s; repeat 3 times (lift it back to the left).\n"
          "     watch 'flow right' (the PC's right) and 'FC right' (the FC's own estimate).\n", flush=True)
    try:
        while now() < seconds:
            s.set_controls(Controls()); time.sleep(0.02)
            if now() - last >= 0.2 and rows:
                last = now(); k = next((r for r in reversed(rows) if r[0] == "k"), None); o = next((r for r in reversed(rows) if r[0] == "o"), None)
                if k and o:
                    print(f"  {now():5.1f} s  roll {math.degrees(k[2]):+5.1f}  pitch {math.degrees(k[3]):+5.1f} deg | height {o[2]:.2f} m  quality {o[5]:3.0f} | "
                          f"flow fwd {o[3]:+.2f}  FC fwd {k[5]:+.2f} | flow right {o[4]:+.2f}  FC right {k[6]:+.2f} m/s", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        s.set_controls(Controls()); time.sleep(0.1); s.stop()
    out = os.path.join(DF, "docs", "status_logs", time.strftime("%Y-%m-%d_%H%M") + "_bench-v47_live.csv")
    open(out, "w").write("kind,t,a,b,c,d,e,f\n" + "\n".join(",".join(str(x) if isinstance(x, str) else f"{x:.4f}" for x in r) for r in rows) + "\n")
    print(f"saved: {out}")
    return 0


def main():
    args = sys.argv[1:]; only = None
    if "--live" in args:
        args.remove("--live")
        return live(args[0] if args else next(iter(sorted(glob.glob("/dev/serial/by-id/usb-Silicon_Labs_CP2102*-port0"))), ""))
    if "--only" in args:
        i = args.index("--only"); only = set(args[i + 1].split(",")) | {"level"}; del args[i:i + 2]
    steps = [st_ for st_ in STEPS if only is None or st_[0] in only or st_[0].rstrip("12") in only]
    port = args[0] if args else next(iter(sorted(glob.glob("/dev/serial/by-id/usb-Silicon_Labs_CP2102*-port0"))), None)
    if not port: print("No ELRS dongle (CP2102) found: plug it in, or give the port."); return 2
    s = Session(port, hz=100.0, allow_arm=False, on_telemetry=on_tel); s.start(); marks = {}; L = []
    try:
        bell("Bench check: props OFF, battery in, drone DISARMED. Waiting for the FC telemetry")
        nxt = 5.0
        while now() < 45 and not any(r[0] == "k" for r in rows):          # after a power-up the FC can take a while before its telemetry flows
            s.set_controls(Controls()); time.sleep(0.02)
            if now() >= nxt:
                nxt += 5.0; tel = s.telemetry; lk = tel.get("link") or {}
                print(f"  {now():4.0f} s: radio link {'UP, LQ ' + str(lk.get('up_lq')) + ' %' if lk.get('up_lq') else 'not linked'}, "
                      f"FC answered: {'yes' if 'flight_mode' in tel or 'battery' in tel else 'not yet'}, state frames: 0", flush=True)
        if not any(r[0] == "k" for r in rows):
            print("No FC state telemetry in 45 s. Radio linked but FC silent: power-cycle the drone (charged battery) and wait 20 s. Radio not linked: check the dongle / receiver."); return 1
        print(f"  FC telemetry flowing after {now():.0f} s", flush=True)
        aid = AidSupport()
        while aid.update(s) not in ("yes", "no", "silent") and now() < 40:
            s.set_controls(Controls()); time.sleep(0.05)
        L.append(f"0 firmware     MSP2_DF_POS_AID answer: {aid.state}  ->  " + {"yes": "PASS (the FC handles the VO aid: v4.7 ST)", "no": "FAIL (the FC does not know the aid: not v4.7)",
                                                                              "silent": "UNCLEAR (no MSP reply over the radio: the GCS will keep the aid OFF)"}.get(aid.state, "UNCLEAR"))
        print(L[-1])
        t = now() + 1.0
        for key, what, _k in steps:
            marks[key] = (t + READY, t + READY + ACT); t += READY + ACT + BACK
        end = t; told = set()
        while now() < end:
            s.set_controls(Controls())
            for key, what, _k in steps:
                a, b = marks[key]
                if key + "r" not in told and now() >= a - READY: told.add(key + "r"); bell(f"GET READY: {what}")
                if key + "g" not in told and now() >= a: told.add(key + "g"); bell(f"GO: {what}")
                if key + "b" not in told and now() >= b: told.add(key + "b"); bell("back to LEVEL, nose away from you, 0.3-0.5 m up, still")
            time.sleep(0.01)
    finally:
        s.set_controls(Controls()); time.sleep(0.1); s.stop()
    bell("bench check finished")

    def att(i, a, b):                                       # i: 2 roll, 3 pitch, 4 yaw
        w = win("k", a, b); return [r[i] for r in w]
    base = marks["level"]; deg = math.degrees
    for key, idx, name, want in (("pitch", 3, "pitch", "nose down"), ("roll", 2, "roll", "right side down")):
        if key not in marks: continue
        a, b = marks[key]; v0 = att(idx, *base); v1 = att(idx, a + 0.8, b)
        if len(v0) < 5 or len(v1) < 5: L.append(f"{'1' if key == 'pitch' else '2'} {name:12s} too few samples"); continue
        d = st.median(v1) - st.median(v0)
        L.append(f"{'1' if key == 'pitch' else '2'} {name:12s} {want}: {deg(d):+.0f} deg  ->  " + ("PASS (+ as the code assumes)" if deg(d) > 8 else "SIGN REVERSED (the code assumes +)" if deg(d) < -8 else "UNCLEAR (tilt more)"))
    a, b = marks.get("yaw", (0.0, 0.0)); w = [r for r in rows if r[0] == "k" and a - 0.5 <= r[1] <= b]; y = unwrap([r[4] for r in w]); tt = [r[1] for r in w]
    if len(y) > 5:                                          # start (before GO) to the end of the turn (before 'back')
        y0 = st.median([v for v, t_ in zip(y, tt) if t_ <= a] or y[:3]); y1 = st.median([v for v, t_ in zip(y, tt) if t_ >= b - 0.4] or y[-3:]); d = deg(y1 - y0); L.append(f"3 yaw          clockwise turn: {d:+.0f} deg  ->  " + ("PASS (clockwise +)" if d > 30 else "SIGN REVERSED (the code assumes clockwise +)" if d < -30 else "UNCLEAR (turn further)"))
    a, b = marks.get("up", (0.0, 0.0)); o0 = [r[2] for r in win("o", a - 0.5, a) if r[2] > 0]; o1 = [r[2] for r in win("o", b - 0.4, b) if r[2] > 0]; k = win("k", a, b)
    if o0 and o1 and len(k) > 5:
        r0, r1 = st.median(o0), st.median(o1); dr = r1 - r0; vz = st.mean(r[7] for r in k)
        L.append(f"4 height       lift: range {r0:.2f} -> {r1:.2f} m ({dr:+.2f} m), EKF vz mean {vz:+.2f} m/s  ->  "
                 + ("PASS" if dr > 0.1 and vz > 0.02 else "VZ SIGN REVERSED (the code assumes up +)" if dr > 0.1 and vz < -0.02 else "UNCLEAR (lift more, slowly)"))
    for axis, keys, fi, ki in (("forward", ("fwd1", "fwd2"), 3, 5), ("right", ("right1", "right2"), 4, 6)):
        fl, ek, q = [], [], []
        for key in keys:
            if key not in marks: continue
            a, b = marks[key]; o = [r for r in win("o", a + 0.2, b + 0.3) if r[5] >= 20]; k = win("k", a + 0.2, b + 0.3)
            if o: fl.append(st.mean(r[fi] for r in o)); q.append(st.mean(r[5] for r in o))
            if k: ek.append(st.mean(r[ki] for r in k))
        n = "5" if axis == "forward" else "6"
        if not any(k_ in marks for k_ in keys): continue
        if not fl: L.append(f"{n} flow {axis:8s} no flow samples with quality >= 20 (floor texture? height 0.3-0.5 m?)"); continue
        ok_f = all(x > 0.03 for x in fl); bad_f = all(x < -0.03 for x in fl); ok_e = ek and all(x > 0.02 for x in ek); bad_e = ek and all(x < -0.02 for x in ek)
        L.append(f"{n} flow {axis:8s} flow {', '.join(f'{x:+.2f}' for x in fl)} m/s, EKF {', '.join(f'{x:+.2f}' for x in ek)} m/s, quality {st.mean(q):.0f}  ->  flow "
                 + ("PASS" if ok_f else "SIGN REVERSED" if bad_f else "UNCLEAR") + ", EKF " + ("PASS" if ok_e else "SIGN REVERSED" if bad_e else "UNCLEAR"))
    L.append("")
    L.append("Not checked here: the VO aid DIRECTION from the real camera: GCS, VO on, disarmed: carry the drone 0.3 m toward the nose: the VISION bar must show a + forward value.")
    os.makedirs(os.path.join(DF, "docs", "status_logs"), exist_ok=True)
    out = os.path.join(DF, "docs", "status_logs", time.strftime("%Y-%m-%d_%H%M") + "_bench-v47_signs.txt")
    open(out, "w").write("\n".join(L) + "\n")
    open(out[:-4] + "_steps.txt", "w").write("".join(f"{k_} GO {a:.2f} s, back {b:.2f} s\n" for k_, (a, b) in marks.items()))
    open(out[:-4] + ".csv", "w").write("kind,t,a,b,c,d,e,f\n" + "\n".join(",".join(str(x) if isinstance(x, str) else f"{x:.4f}" for x in r) for r in rows) + "\n")
    print("\n" + "\n".join(L) + f"\nsaved: {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
