#!/usr/bin/env python3
"""Hybrid flight: the FC runs the height loop (take-off creep, hold, landing: firmware flight/height_hold*.c), the PC only arms, sends the target height + hover reference and watches.

THIS ARMS THE DRONE: the motors will spin with the props on. Tether it, keep hands and cables clear, Ctrl-C = SAFE.
Needs the firmware with the FC hold + the 0xD6 state frame that carries the hold status (builds/2026-11-13_fc-hybrid_*), AUX5 mode USER1 configured (aux 5 40 4 1100 2100).

  python3 tools_scripts/fc_hold_run.py --sim               # against the simulator with the firmware's own control law (arms only the simulator)
  python3 tools_scripts/fc_hold_run.py --hold 8            # real drone: 0.5 m, hold 8 s, land

Sequence: arm (gyro calibration, idle) -> 4 s idle check -> request take-off (AUX5 target, AUX6 hover reference, throttle stick at minimum) -> the FC creeps up, lifts, holds ->
after --hold seconds LAND (the FC descends at <= 0.12 m/s and ramps to idle) -> disarm. The PC never touches the throttle while the FC hold is active.
PC rules: tilt > 15 deg, link LQ < 50 or a fault = SAFE at once; height above --abort-height = the FC's gentle LAND (never a disarm in the air); the FC hold letting go (fault) while
above 0.25 m = the PC holds a descending throttle until it is low, then SAFE; no hold within 12 s of the request = SAFE (nothing has lifted)."""
import argparse, dataclasses, glob, os, sys, time

sys.path.insert(0, os.path.expanduser("~/Music/DroneForge/software/pc"))
from nimbus_core.core import NimbusCore  # noqa: E402
from nimbus_core import topics as T  # noqa: E402
from nimbus_pc.mapper import height_to_aux5, AUX5_LAND  # noqa: E402
from nimbus_pc.sender import CRSF_MIN  # noqa: E402
from nimbus_pc.mapper import throttle01_to_crsf  # noqa: E402

WARN = "THIS ARMS THE DRONE: the motors will spin with the props on. Tether it, keep hands and cables clear, Ctrl-C = SAFE."


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="", help="radio port (default: the CP2102 in /dev/serial/by-id)")
    ap.add_argument("--height", type=float, default=0.5, help="target height, m")
    ap.add_argument("--hold", type=float, default=8.0, help="seconds to hold before landing")
    ap.add_argument("--abort-height", type=float, default=0.8, help="the FC lands gently above this height, m")
    ap.add_argument("--hover-ref", type=float, default=0.0, help="hover-reference throttle sent to the FC (0 = the saved thr_hover0)")
    ap.add_argument("--countdown", type=float, default=10.0)
    ap.add_argument("--takeoff-timeout", type=float, default=90.0, help="seconds the FC may creep before the PC gives up (one creep takes 17-20 s; after a sank-back the FC creeps again)")
    ap.add_argument("--sim", action="store_true", help="simulator with the firmware's control law (no hardware)")
    ap.add_argument("--sim-hover", type=float, default=0.72, help="--sim only: the simulated drone's true hover throttle")
    a = ap.parse_args(argv)
    print(WARN)
    core = NimbusCore(); d = core.director
    tick = lambda: (core.tick(), time.sleep(0.01))
    def spin(sec):
        end = time.monotonic() + sec
        while time.monotonic() < end: tick()
    def say(msg): print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)
    st = {"hold_active": False, "hold_fault": False, "hold_mode": 0, "t": 0.0, "range": None, "lq": None, "tilt": 0.0}
    def on_tel(m):
        n, dd = m
        if n == "kinematic" and "hold_active" in dd: st.update(hold_active=dd["hold_active"], hold_fault=dd["hold_fault"], hold_mode=dd["hold_mode"], t=time.monotonic())
        elif n == "optrange": st["range"] = dd["distance_m"]
        elif n == "link": st["lq"] = dd.get("up_lq")
    core.bus.subscribe(T.TELEMETRY, on_tel)
    try:
        if a.sim:
            from nimbus_core.fc_hold_model import FcHoldModel
            from nimbus_core import settings as S
            core.connect_sim(params=S.plant_params(dataclasses.replace(core.autonomy.settings, thr_hover0=a.sim_hover)))
            core.sess.fc_hold = FcHoldModel()                 # the firmware's own parameters (thr_max 0.85)
        else:
            port = a.port or next(iter(glob.glob("/dev/serial/by-id/*CP2102*-port0")), "")
            if not port: sys.exit("no CP2102 radio port found: plug in the radio dongle or pass --port")
            say(f"connecting to {port}"); core.connect(port)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 25.0:
            tick(); pin = d.preflight_input()
            if (pin.get("link_lq") or 0) > 0 and pin.get("rest_tilt_deg") is not None and pin.get("range_m") is not None and pin.get("battery_v") is not None: break
        else:
            sys.exit(f"no telemetry after 25 s (link {pin.get('link_lq')}, attitude {pin.get('rest_tilt_deg')}, range {pin.get('range_m')}, battery {pin.get('battery_v')}): not arming")
        spin(2.0)
        h = min(max(a.height, 0.2), 1.5); ref = a.hover_ref or core.autonomy.settings.thr_hover0
        if not 0.3 <= ref <= 0.9: sys.exit(f"hover reference {ref:.2f} is outside 0.3..0.9: not arming")
        ground = st["range"] if st["range"] is not None else 0.0
        say(f"FC hold: target {h:.2f} m, hover reference {ref:.2f}, gentle landing above {a.abort_height:.2f} m (range at rest {ground:.2f} m)")
        for c in d.preflight():
            if not c.ok: say(f"{'BLOCKING' if c.blocking else 'warning '}: {c.label}: {c.detail}")
        blocking = [c for c in d.preflight() if not c.ok and c.blocking]
        if blocking: sys.exit("cannot start: " + "; ".join(f"{c.label}: {c.detail}" for c in blocking))
        say(f"arming in {a.countdown:.0f} s: Ctrl-C to cancel"); spin(a.countdown)

        r = d.arm(); say(f"ARM: {r.message}")
        if not r.ok: sys.exit("arm refused")
        t0 = time.monotonic()
        while d.state().phase != "armed" and time.monotonic() - t0 < 25.0: tick()
        if d.state().phase != "armed": raise RuntimeError(f"not armed after 25 s: {d.state().message}")
        say("ARMED (motors at idle): check all four turn"); spin(4.0)
        ground = st["range"] if st["range"] is not None else ground

        aux5, aux6 = height_to_aux5(h), throttle01_to_crsf(ref)
        core.set_fc_hold(aux5, aux6); t_req = time.monotonic(); say("FC take-off requested (throttle stick at minimum: the FC creeps up by itself)")
        phase = "takeoff"; t_hold = None; last = 0.0; landing = False; fallback = False; tilt_bad = 0
        while True:
            tick(); now = time.monotonic()
            rng = st["range"]; height = (rng - ground) if rng is not None else 0.0
            pin = d.preflight_input(); tilt = pin.get("rest_tilt_deg") or 0.0
            if now - last >= 1.0:
                last = now; say(f"{phase:9s} height {height:5.2f} m  hold {'ON ' if st['hold_active'] else 'off'} mode {st['hold_mode']} fault {int(st['hold_fault'])}  tilt {tilt:4.1f}  LQ {st['lq']}")
            if tilt > 15.0: raise RuntimeError(f"abort: tilt {tilt:.1f} deg")
            if st["lq"] is not None and st["lq"] < 50: raise RuntimeError(f"abort: link LQ {st['lq']}")
            if d.state().phase in ("fault", "disconnected"): raise RuntimeError(f"abort: {d.state().phase}")
            if phase == "takeoff":
                if st["hold_active"] and st["hold_mode"] == 0 and height > 0.15: phase = "holding"; t_hold = now; say("HOLDING (the FC climbed to the target and holds there)")
                elif now - t_req > a.takeoff_timeout: raise RuntimeError(f"no lift-off within {a.takeoff_timeout:.0f} s of the request")
            if st["hold_fault"] and not fallback and not landing and phase != "landed":
                if height > 0.25:
                    fallback = True; core.set_fc_hold(CRSF_MIN, CRSF_MIN, 0.9 * ref); say(f"FC HOLD FAULT at {height:.2f} m: the PC sends a descending throttle {0.9 * ref:.2f} until low")
                else: raise RuntimeError(f"FC hold fault at {height:.2f} m")
            if fallback and height < 0.25: raise RuntimeError("FC hold fault: low enough, SAFE")
            if height > a.abort_height and not landing and not fallback:
                core.set_fc_hold(AUX5_LAND, aux6); landing = True; phase = "landing"; say(f"HEIGHT {height:.2f} m above the {a.abort_height:.2f} m limit: the FC LANDS slowly")
            if phase == "holding" and not landing and now - t_hold >= a.hold:
                core.set_fc_hold(AUX5_LAND, aux6); landing = True; phase = "landing"; say("LAND requested: the FC descends and ramps to idle")
            if landing and not st["hold_active"] and height < 0.12 and now - t_req > 5.0:
                phase = "landed"; say("on the ground (the FC ended its landing)"); break
        core.clear_fc_hold(); d.safe(); spin(2.0)
        say("done: replug the FC USB, wait 90 s, then `df.py logs`")
        return 0
    except KeyboardInterrupt:
        say("Ctrl-C: SAFE"); return 1
    except Exception as e:
        say(f"{e}: SAFE"); return 2
    finally:
        try: core.clear_fc_hold(); d.safe(); spin(1.0)
        finally: core.disconnect()


if __name__ == "__main__":
    sys.exit(main())
