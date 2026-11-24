#!/usr/bin/env python3
"""Tethered run without the GCS: arm, take off to --height, hold --hold seconds, land. Same flight director as the GCS (PC controller, FC firmware v3).

THIS ARMS THE DRONE: the motors will spin with the props on. Tether it, keep hands and cables clear, FC USB unplugged, battery connected.
Ctrl-C at any time = SAFE (disarm, controls idle). Above --abort-height the drone LANDS (the controller's slow controlled descent, never a disarm in the air).
Abort rules (SAFE at once): tilt > 15 deg, link LQ < 50, fault.
Settings are applied for this run only (not saved). The GCS recorder writes the CSV and events to docs/flights/.

  python3 tools_scripts/tethered_run.py                      # real drone, CP2102 radio port, 0.5 m target, 0.8 m ceiling
  python3 tools_scripts/tethered_run.py --sim                # same script against the simulator (arms only the simulator)
"""
import argparse, dataclasses, glob, os, sys, time

sys.path.insert(0, os.path.expanduser("~/Music/DroneForge/software/pc"))
from nimbus_core.core import NimbusCore  # noqa: E402

WARN = "THIS ARMS THE DRONE: the motors will spin with the props on. Tether it, keep hands and cables clear, Ctrl-C = SAFE."


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="", help="radio port (default: the CP2102 in /dev/serial/by-id)")
    ap.add_argument("--height", type=float, default=0.5, help="target height, m")
    ap.add_argument("--ceiling", type=float, default=0.8, help="tether limit, m")
    ap.add_argument("--hold", type=float, default=20.0, help="seconds to hold at the target")
    ap.add_argument("--abort-height", type=float, default=0.7, help="SAFE above this height, m")
    ap.add_argument("--countdown", type=float, default=10.0, help="seconds before arming; Ctrl-C cancels")
    ap.add_argument("--sim", action="store_true", help="run against the simulator (no hardware)")
    a = ap.parse_args()
    print(WARN)
    core = NimbusCore(); d = core.director
    tick = lambda: (core.tick(), time.sleep(0.02))
    def spin(sec):
        end = time.monotonic() + sec
        while time.monotonic() < end: tick()
    def say(msg): print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)

    try:
        if a.sim:
            core.connect_sim()
        else:
            port = a.port or next(iter(glob.glob("/dev/serial/by-id/*CP2102*-port0")), "")
            if not port: sys.exit("no CP2102 radio port found: plug in the radio dongle or pass --port")
            say(f"connecting to {port}"); core.connect(port)
        t0 = time.monotonic()                      # never arm blind: wait for link, attitude, range and battery data (up to 25 s)
        while time.monotonic() - t0 < 25.0:
            tick(); pin = d.preflight_input()
            if (pin.get("link_lq") or 0) > 0 and pin.get("rest_tilt_deg") is not None and pin.get("range_m") is not None and pin.get("battery_v") is not None: break
        else:
            sys.exit(f"no telemetry after 25 s (link {pin.get('link_lq')}, attitude {pin.get('rest_tilt_deg')}, range {pin.get('range_m')}, battery {pin.get('battery_v')}): not arming")
        spin(2.0)
        h = min(a.height, a.ceiling)
        au = d._au
        res = d.apply_settings(dataclasses.replace(au.settings, hover_height=h, ceiling_m=a.ceiling, profile="real"), persist=False)      # the controller tuning fitted to the real flights (this run only)
        if not res.ok: sys.exit(f"settings refused: {res.message}")
        say(f"target {h:.2f} m, ceiling {a.ceiling:.2f} m, real-flight controller profile (this run only)")
        for c in d.preflight():
            if not c.ok: say(f"{'BLOCKING' if c.blocking else 'warning '}: {c.label}: {c.detail}")
        blocking = [c for c in d.preflight() if not c.ok and c.blocking]
        if blocking: sys.exit("cannot start: " + "; ".join(f"{c.label}: {c.detail}" for c in blocking))
        say(f"arming in {a.countdown:.0f} s: Ctrl-C to cancel")
        spin(a.countdown)

        r = d.arm(); say(f"ARM: {r.message}")
        if not r.ok: sys.exit("arm refused")
        t0 = time.monotonic()
        while d.state().phase != "armed" and time.monotonic() - t0 < 25.0: tick()
        if d.state().phase != "armed": raise RuntimeError(f"not armed after 25 s: {d.state().message}")
        say("ARMED (motors at idle): check all four turn")
        spin(4.0)

        r = d.takeoff(h); say(f"TAKE OFF: {r.message}")
        if not r.ok: raise RuntimeError(r.message)
        t_take = time.monotonic(); t_hold = None; last = 0.0; landing = False
        while True:
            tick(); s = d.state(); now = time.monotonic()
            tilt = d.preflight_input().get("rest_tilt_deg") or 0.0
            if now - last >= 1.0:
                last = now
                say(f"{s.phase:9s} height {s.height_m if s.height_m is not None else float('nan'):.2f} m  target {s.target_m:.2f}  tilt {tilt:.1f}  LQ {s.link_lq}  batt {s.battery_v if s.battery_v is None else round(s.battery_v, 2)}")
            if s.phase == "holding" and t_hold is None: t_hold = now; say("HOLDING")
            if s.phase in ("fault", "disconnected"): raise RuntimeError(f"abort: {s.phase}: {s.message}")
            if tilt > 15.0: raise RuntimeError(f"abort: tilt {tilt:.1f} deg")
            if s.height_m is not None and s.height_m > a.abort_height and not landing:
                r = d.land(); landing = True; t_hold = now + 9999            # too high: a slow controlled landing, not a disarm in the air
                say(f"HEIGHT {s.height_m:.2f} m above the {a.abort_height:.2f} m limit: LANDING slowly ({r.message})")
            if s.link_lq is not None and s.link_lq < 50: raise RuntimeError(f"abort: link LQ {s.link_lq}")
            if s.phase in ("landed", "ready") and now - t_take > 5.0: say("on the ground"); break
            if t_hold is None and now - t_take > 60.0: raise RuntimeError("no hold within 60 s of take-off")
            if t_hold is not None and now - t_hold >= a.hold and s.phase == "holding" and not landing:
                r = d.land(); say(f"LAND: {r.message}"); landing = True; t_hold = now + 9999
        spin(3.0)
        say("done: replug the FC USB, wait 90 s, then `df.py logs`")
        return 0
    except KeyboardInterrupt:
        say("Ctrl-C: SAFE"); return 1
    except Exception as e:
        say(f"{e}: SAFE"); return 2
    finally:
        try: d.safe(); spin(1.0)
        finally: core.disconnect()


if __name__ == "__main__":
    sys.exit(main())
