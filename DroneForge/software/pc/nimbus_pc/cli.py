"""Command line front end: python3 -m nimbus_pc <command> ... (run with -h for each command)."""
from __future__ import annotations

import argparse
import glob
import math
import sys
import time
from typing import Optional

from . import commands
from .crsf import TYPE_OPTRANGE
from .gamepad import GamepadInput
from .sender import Controls
from .session import Session
from .watchdog import NAMES as WATCHDOG_NAMES


def _ports() -> list[str]:
    return sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*") + glob.glob("/dev/serial/by-id/*"))


def _open(args, **kw) -> Session:
    s = Session(args.port, **kw)
    try:
        s.start()
    except OSError as e:
        print(f"cannot open {args.port}: {e.strerror}. Candidate ports: {', '.join(_ports()) or 'none found'}. "
              f"(Use the serial port that feeds the ELRS dongle's UART; the flight controller's USB port is MSP, "
              f"not CRSF.)", file=sys.stderr)
        raise SystemExit(2)
    return s


def _summary(t: dict) -> str:
    parts = []
    if "drone_linked" in t:
        parts.append("drone " + ("linked" if t["drone_linked"]["is_linked"] else "NOT linked"))
    if "link" in t:
        parts.append(f"LQ {t['link']['up_lq']}% RSSI {t['link']['up_rssi1_dbm']}dBm")
    if "battery" in t:
        parts.append(f"{t['battery']['voltage_v']:.1f}V")
    if "attitude" in t:
        a = t["attitude"]
        parts.append(f"P{math.degrees(a['pitch_rad']):+.0f} R{math.degrees(a['roll_rad']):+.0f} Y{math.degrees(a['yaw_rad']):+.0f}")
    if "flight_mode" in t:
        parts.append(t["flight_mode"]["mode"])
    if "optrange" in t:
        o = t["optrange"]
        parts.append(f"range {o['distance_m']:.3f}m flowQ {o['flow_quality']}")
    return " | ".join(parts) or "(no telemetry yet)"


def cmd_ports(args) -> int:
    for p in _ports():
        print(p)
    return 0


def cmd_send(args) -> int:
    pad = GamepadInput(args.gamepad) if args.gamepad else None
    watch = args.watch_optrange
    s = _open(args, hz=args.hz, allow_arm=args.allow_arm, ping=not args.no_ping, watch_optrange=watch,
              log_path=args.log)
    t0, last = time.monotonic(), 0.0
    try:
        while args.duration <= 0 or time.monotonic() - t0 < args.duration:
            t = time.monotonic() - t0
            if pad:
                pad.poll()
                if pad.connected and pad.ready:
                    s.set_controls(pad.controls())
                # otherwise stop updating: the sender's stale-input failsafe takes over
            elif args.demo == "sweep":
                s.set_controls(Controls(roll=0.5 * math.sin(t), pitch=0.5 * math.sin(0.7 * t),
                                        yaw=0.5 * math.sin(0.5 * t), arm=args.arm and t >= args.arm_after))
            else:
                s.set_controls(Controls(arm=args.arm and t >= args.arm_after))
            if time.monotonic() - last > 1.0:
                last = time.monotonic()
                wd = WATCHDOG_NAMES[s.watchdog.state] if s.watchdog else "off"
                print(f"[{t:5.1f}s] frames {s.sender.frames_sent} pings {s.sender.pings_sent} "
                      f"failsafe {'ON' if s.sender.failsafe_active else 'off'} watchdog {wd} | {_summary(s.telemetry)}",
                      flush=True)
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        s.stop()
    print(f"sent {s.sender.frames_sent} frames + {s.sender.pings_sent} pings, max interval "
          f"{s.sender.max_interval_s * 1000:.1f} ms, telemetry frames {s.reader.frame_count} "
          f"(crc errors {s.reader.parser.crc_errors})")
    return 0


def cmd_monitor(args) -> int:
    s = _open(args, listen_only=not args.send_idle, ping=args.send_idle, log_path=args.log)
    t0 = time.monotonic()
    try:
        while args.duration <= 0 or time.monotonic() - t0 < args.duration:
            if args.send_idle:
                s.set_controls(Controls())
            print(f"[{time.monotonic() - t0:5.1f}s] {_summary(s.telemetry)}", flush=True)
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        s.stop()
    return 0


def cmd_live(args) -> int:
    from . import live
    st = live.LiveState()
    pad = GamepadInput(args.gamepad) if args.gamepad else None
    sess = Session(args.port, hz=args.hz, allow_arm=args.allow_arm, ping=not args.listen, listen_only=args.listen,
                   watch_optrange=args.watch_optrange, log_path=args.log, on_telemetry=st.note_decoded, on_frame=st.note_in)
    sess.link.on_write = st.note_out
    try:
        sess.start()
    except OSError as e:
        print(f"cannot open {args.port}: {e.strerror}. Candidate ports: {', '.join(_ports()) or 'none found'}", file=sys.stderr)
        return 2
    source = "listen" if args.listen else "gamepad" if pad else args.demo
    t0 = time.monotonic()
    prev = {"arm": False, "fs": False, "wd": 0}

    def drive() -> None:
        t = time.monotonic() - t0
        if args.listen:
            return
        if pad:
            pad.poll()
            c = pad.controls() if (pad.connected and pad.ready) else Controls()
            if not pad.connected and prev.get("pad", True):
                st.event("gamepad LOST -> safe frames")
            prev["pad"] = pad.connected
        elif args.demo == "sweep":
            c = Controls(roll=0.5 * math.sin(2 * t), pitch=0.5 * math.sin(1.4 * t), yaw=0.5 * math.sin(t))
        else:
            c = Controls()
        sess.set_controls(c)
        arm = bool(c.arm and args.allow_arm)
        if arm != prev["arm"]:
            st.event("arm channel " + ("HIGH" if arm else "low")); prev["arm"] = arm
        if sess.sender.failsafe_active != prev["fs"]:
            st.event("stale-input failsafe " + ("ON" if sess.sender.failsafe_active else "off")); prev["fs"] = sess.sender.failsafe_active
        if sess.watchdog and sess.watchdog.state != prev["wd"]:
            st.event("sensor watchdog -> " + WATCHDOG_NAMES[sess.watchdog.state]); prev["wd"] = sess.watchdog.state

    try:
        live.run(sess, st, port=args.port, duration=args.duration, source=source, set_controls=drive, raw_lines=args.raw,
                 fps=args.fps, color=not args.no_color, clear=not args.no_clear)
    finally:
        sess.stop()
        if pad:
            pad.close()
    return 0


def cmd_hover_sim(args) -> int:
    from .hover import runner
    if args.list:
        for k, (desc, _, _) in runner.SCENARIOS.items():
            print(f"{k:15s} {desc}")
        return 0
    names = list(runner.SCENARIOS) if args.scenario == "all" else [args.scenario]
    for name in names:
        res = runner.run(name, duration=args.duration, seed=args.seed, land_at=args.land_at)
        print(runner.summary(res)); print()
        if args.csv:
            runner.write_csv(res, args.csv if len(names) == 1 else f"{args.csv}.{name}.csv")
        if args.png:
            from .hover import plot
            plot.save_png(res, args.png if len(names) == 1 else f"{args.png}.{name}.png")
    return 0


def cmd_doctor(args) -> int:
    from . import doctor
    try:
        stats = doctor.run(args.port, fc_port=args.fc_port, duration=args.duration, hz=args.hz, msp=not args.no_msp)
    except OSError as e:
        print(f"cannot open a port: {e}. Candidate ports: {', '.join(_ports()) or 'none found'}", file=sys.stderr)
        return 2
    if stats.get("aborted"):
        print(f"doctor aborted: {stats['aborted']}", file=sys.stderr)
        return 2
    checks = doctor.evaluate(stats)
    print(doctor.report(checks, as_json=args.json))
    return doctor.verdict(checks)


def cmd_bind(args) -> int:
    uid = bytes.fromhex(args.uid) if args.uid else None
    s = _open(args, listen_only=True, ping=False)
    try:
        s.bind(phrase=args.phrase, uid=uid)
        print("sent " + ("set-bind-UID + " if (args.phrase or uid) else "") + "receiver bind command")
        time.sleep(1.0)
    finally:
        s.stop()
    return 0


def cmd_vrx(args) -> int:
    s = _open(args, listen_only=True, ping=False)
    try:
        if args.action == "scan":
            s.send_frame(commands.encode_vrx_scan())
        else:
            s.send_frame(commands.encode_vrx_connect(args.band, args.channel))
        print(f"sent video-receiver {args.action} (Nimbus-specific command; stock ELRS is not expected to react)")
        time.sleep(0.5)
    finally:
        s.stop()
    return 0


def cmd_msp(args) -> int:
    s = _open(args, listen_only=True, ping=False)
    try:
        cmd = commands.MSP_BOARD_INFO if args.what == "board-info" else commands.MSP_VTX_CONFIG
        reply = s.request_msp(cmd, timeout=args.timeout)
    finally:
        s.stop()
    if reply is None:
        print("no MSP response (timeout)")
        return 1
    if cmd == commands.MSP_BOARD_INFO:
        print(commands.decode_board_info(reply["payload"]))
    else:
        print({"cmd": reply["cmd"], "payload_hex": reply["payload"].hex()})
    return 0


def cmd_vtx(args) -> int:
    s = _open(args, listen_only=True, ping=False)
    try:
        if args.action == "set":
            frames = commands.encode_msp_set_vtx_config_power(args.band, args.channel, args.power)
            for f in frames:
                s.send_frame(f)
                time.sleep(0.02)
            print(f"sent MSP_SET_VTX_CONFIG: band {args.band.upper()} channel {args.channel} power level {args.power} "
                  f"({len(frames)} CRSF frames). Power is the VTX table's level index, not mW.")
            return 0
        reply = s.request_msp(commands.MSP_VTX_CONFIG, timeout=args.timeout)
    finally:
        s.stop()
    if reply is None:
        print("no MSP response (timeout)")
        return 1
    print(commands.decode_vtx_config(reply["payload"]) or {"payload_hex": reply["payload"].hex()})
    return 0


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(prog="nimbus_pc", description="NimbusOS-style PC side for an ELRS dongle (props off!).")
    sub = ap.add_subparsers(dest="command", required=True)
    sub.add_parser("ports", help="list candidate serial ports").set_defaults(fn=cmd_ports)

    p = sub.add_parser("send", help="send RC frames at ~100 Hz and show telemetry")
    p.add_argument("--port", required=True)
    p.add_argument("--hz", type=float, default=100.0)
    p.add_argument("--duration", type=float, default=10.0, help="seconds, 0 = until Ctrl-C")
    p.add_argument("--demo", choices=["idle", "sweep"], default="idle")
    p.add_argument("--arm-after", type=float, default=0.0, help="demo modes: only raise the arm switch after this many seconds")
    p.add_argument("--gamepad", metavar="DEV", help="drive from a joystick, e.g. /dev/input/js1 (Boxer in USB Joystick mode)")
    p.add_argument("--allow-arm", action="store_true", help="let the arm channel go high (REMOVE PROPS FIRST)")
    p.add_argument("--arm", action="store_true", help="demo modes: request arming (needs --allow-arm)")
    p.add_argument("--no-ping", action="store_true")
    p.add_argument("--watch-optrange", action="store_true", help="sensor watchdog on the 0xD3 range/flow frame")
    p.add_argument("--log", metavar="CSV", help="write telemetry to a CSV file")
    p.set_defaults(fn=cmd_send)

    p = sub.add_parser("monitor", help="show telemetry (listen only unless --send-idle)")
    p.add_argument("--port", required=True)
    p.add_argument("--duration", type=float, default=10.0)
    p.add_argument("--send-idle", action="store_true", help="also send disarmed idle frames and pings")
    p.add_argument("--log", metavar="CSV")
    p.set_defaults(fn=cmd_monitor)

    p = sub.add_parser("live", help="real-time view of everything flowing over the link (telemetry in, commands out)")
    p.add_argument("--port", required=True, help="serial port feeding the dongle's CRSF UART")
    p.add_argument("--duration", type=float, default=0.0, help="seconds, 0 = until Ctrl-C")
    p.add_argument("--listen", action="store_true", help="only listen: send nothing (the TX may not forward telemetry without a handset)")
    p.add_argument("--demo", choices=["idle", "sweep"], default="idle", help="what to send when not using a gamepad (never arms)")
    p.add_argument("--gamepad", metavar="DEV", help="drive from a joystick, e.g. /dev/input/js1")
    p.add_argument("--allow-arm", action="store_true", help="let the arm channel go high (REMOVE PROPS FIRST)")
    p.add_argument("--watch-optrange", action="store_true", help="enable the sensor watchdog on 0xD3")
    p.add_argument("--hz", type=float, default=100.0)
    p.add_argument("--fps", type=float, default=5.0, help="screen refresh rate")
    p.add_argument("--raw", type=int, default=0, metavar="N", help="also list the last N raw frames (hex), both directions")
    p.add_argument("--no-color", action="store_true")
    p.add_argument("--no-clear", action="store_true", help="append frames instead of redrawing (for logs and pipes)")
    p.add_argument("--log", metavar="CSV", help="also write decoded telemetry to a CSV file")
    p.set_defaults(fn=cmd_live)

    p = sub.add_parser("hover-sim", help="simulate the hover-hold controller (no hardware, nothing flies)")
    p.add_argument("--scenario", default="nominal", help="name, or 'all' (see --list)")
    p.add_argument("--list", action="store_true", help="list the scenarios")
    p.add_argument("--duration", type=float, default=40.0)
    p.add_argument("--land-at", type=float, default=30.0, help="seconds when a landing is requested")
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--csv", metavar="FILE", help="write the run (10 Hz) to a CSV")
    p.add_argument("--png", metavar="FILE", help="draw altitude, position and throttle to a PNG (needs PyQt6)")
    p.set_defaults(fn=cmd_hover_sim)

    p = sub.add_parser("doctor", help="disarmed health check of the whole chain (dongle, RF link, telemetry, FC)")
    p.add_argument("--port", required=True, help="serial port feeding the dongle's CRSF UART")
    p.add_argument("--fc-port", help="flight controller USB port (read-only MSP): adds FC, channel and arming checks")
    p.add_argument("--duration", type=float, default=25.0, help="seconds to run (default 25; the receiver can need ~5-25 s to connect)")
    p.add_argument("--hz", type=float, default=100.0)
    p.add_argument("--no-msp", action="store_true", help="skip the MSP-over-CRSF board-info request")
    p.add_argument("--json", action="store_true", help="machine-readable output")
    p.set_defaults(fn=cmd_doctor)

    p = sub.add_parser("bind", help="send the receiver bind command (optionally set the bind UID first)")
    p.add_argument("--port", required=True)
    g = p.add_mutually_exclusive_group()
    g.add_argument("--phrase", help="ELRS bind phrase")
    g.add_argument("--uid", help="12 hex digits (6 bytes)")
    p.set_defaults(fn=cmd_bind)

    p = sub.add_parser("vrx", help="Nimbus-specific video receiver commands")
    p.add_argument("--port", required=True)
    p.add_argument("action", choices=["scan", "connect"])
    p.add_argument("--band", default="A", help="A B E F R L")
    p.add_argument("--channel", type=int, default=1)
    p.set_defaults(fn=cmd_vrx)

    p = sub.add_parser("vtx", help="read or set the drone's VTX through MSP-over-CRSF")
    p.add_argument("--port", required=True)
    p.add_argument("action", choices=["get", "set"])
    p.add_argument("--band", default="A", help="A B E F R L (set)")
    p.add_argument("--channel", type=int, default=1, help="1..8 (set)")
    p.add_argument("--power", type=int, default=1, help="power-level index of the VTX table, not mW (set)")
    p.add_argument("--timeout", type=float, default=2.0)
    p.set_defaults(fn=cmd_vtx)

    p = sub.add_parser("msp", help="MSP-over-CRSF request to the flight controller")
    p.add_argument("--port", required=True)
    p.add_argument("what", choices=["board-info", "vtx-config"])
    p.add_argument("--timeout", type=float, default=2.0)
    p.set_defaults(fn=cmd_msp)
    return ap


def main(argv: Optional[list] = None) -> None:
    args = build_parser().parse_args(argv)
    raise SystemExit(args.fn(args))
