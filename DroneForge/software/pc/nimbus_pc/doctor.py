"""`nimbus_pc doctor`: one-shot health check of the whole PC -> dongle -> RF -> receiver -> FC chain.

Disarmed only: the arm channel is never raised (allow_arm=False). With --fc-port the flight controller's USB port is read
with read-only MSP requests (status, RC channels, arming-disable flags) and the run aborts if the FC reports armed.
"""
from __future__ import annotations

import json
import math
import os
import select
import statistics as st
import struct
import termios
import time
from dataclasses import dataclass
from typing import Callable, Optional

from . import commands
from .sender import Controls
from .session import Session

PASS, WARN, FAIL, INFO = "PASS", "WARN", "FAIL", "INFO"

ARMING_FLAGS = ["NOGYRO", "FAILSAFE", "RXLOSS", "BADRX", "BOXFAILSAFE", "RUNAWAY", "CRASH", "THROTTLE", "ANGLE", "BOOTGRACE",
                "NOPREARM", "LOAD", "CALIB", "CLI", "CMS", "BST", "MSP", "PARALYZE", "GPS", "RESCUE_SW", "RPMFILTER",
                "REBOOT_REQD", "DSHOT_BBANG", "NO_ACC_CAL", "MOTOR_PROTO", "ARMSWITCH"]
FLIGHT_TELEMETRY = ("battery", "attitude", "flight_mode")
NIMBUS_ONLY = ("drone_linked", "fc_uid", "vrx_top_channels", "rangefinder", "raw_imu", "custom_d2", "custom_d4",
               "custom_d5", "custom_e4")
MSP_READ_ONLY = (101, 105, 150)


@dataclass
class Check:
    status: str
    name: str
    detail: str


# ----------------------------------------------------------------------------- flight controller (read-only MSP v1)
class FcMsp:
    def __init__(self, path: str):
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
        a = termios.tcgetattr(self.fd)
        a[0] = a[1] = a[3] = 0
        a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        a[4] = a[5] = termios.B115200
        a[6][termios.VMIN], a[6][termios.VTIME] = 0, 1
        termios.tcsetattr(self.fd, termios.TCSANOW, a)
        termios.tcflush(self.fd, termios.TCIOFLUSH)

    def close(self) -> None:
        os.close(self.fd)

    def request(self, cmd: int, timeout: float = 0.4) -> Optional[bytes]:
        assert cmd in MSP_READ_ONLY, "doctor only sends read-only MSP requests"
        os.write(self.fd, b"$M<" + bytes([0, cmd, cmd]))
        buf, end = b"", time.monotonic() + timeout
        while time.monotonic() < end:
            if select.select([self.fd], [], [], 0.02)[0]:
                buf += os.read(self.fd, 512)
            i = buf.find(b"$M>")
            if i >= 0 and len(buf) >= i + 6 and len(buf) >= i + 6 + buf[i + 3]:
                return buf[i + 5:i + 5 + buf[i + 3]]
        return None

    def armed(self) -> Optional[bool]:
        p = self.request(101)
        if not p or len(p) < 10:
            return None
        return bool(struct.unpack_from("<I", p, 6)[0] & 1)

    def arming_disable_mask(self) -> Optional[int]:
        p = self.request(150)
        if not p or len(p) < 22:
            return None
        idx = 16 + p[15]
        if len(p) < idx + 5:
            return None
        return struct.unpack_from("<I", p, idx + 1)[0]

    def rc(self) -> Optional[tuple]:
        p = self.request(105)
        return struct.unpack(f"<{len(p) // 2}H", p) if p else None


# ----------------------------------------------------------------------------- evaluation (pure, unit-tested)
def _med(v):
    return st.median(v) if v else None


def evaluate(s: dict) -> list[Check]:
    """s: stats collected by run(); see the keys used below."""
    out: list[Check] = []
    add = lambda status, name, detail: out.append(Check(status, name, detail))

    di = s.get("device_info")
    if di:
        add(PASS, "dongle answers CRSF ping", f"'{di.get('name')}' ({di.get('param_count')} parameters)")
    else:
        add(FAIL, "dongle answers CRSF ping", "no DEVICE_INFO reply: check GPIO20/21 wiring (adapter TX -> module RX, crossed), "
                                              "common ground, 115200 baud")

    dur, hz = s["duration"], s["hz"]
    rate = s["frames_sent"] / dur if dur else 0
    mi = s["max_interval_s"] * 1000
    detail = f"{rate:.0f} frames/s (target {hz:.0f}), max gap {mi:.1f} ms"
    if rate >= 0.9 * hz and mi <= 25:
        add(PASS, "RC frame rate and jitter", detail)
    elif rate >= 0.8 * hz and mi <= 60:
        add(WARN, "RC frame rate and jitter", detail + " (busy PC?)")
    else:
        add(FAIL, "RC frame rate and jitter", detail)

    tel, crc = s["telemetry_frames"], s["crc_errors"]
    if crc == 0 or (tel and crc / tel < 0.001):
        add(PASS, "no CRC errors", f"{tel} telemetry frames parsed" + (f", {crc} stray CRC error(s) (noise)" if crc else ""))
    elif tel and crc / max(tel, 1) < 0.005:
        add(WARN, "no CRC errors", f"{crc} CRC errors in {tel} frames")
    else:
        add(FAIL, "no CRC errors", f"{crc} CRC errors in {tel} frames (cable, baud, or noise)")

    link = s.get("link", {})
    lq = link.get("up_lq", [])
    if not lq:
        add(FAIL, "radio link to the receiver", "no link-statistics frames: TX not transmitting (Wi-Fi hotspot up = radio init "
                                                "failed) or receiver unpowered / different binding phrase / different band")
    else:
        ts = link.get("t") or [0.0] * len(lq)
        first = next((i for i, q in enumerate(lq) if q >= 50), None)
        if first is None:
            add(FAIL, "radio link to the receiver", f"never connected in {dur:.0f} s (LQ stayed 0): receiver searching RF modes "
                                                    "(wait up to ~30 s after a TX restart), wrong binding phrase, band or power?")
        else:
            t_up = ts[first]
            steady = [i for i in range(first, len(lq)) if ts[i] >= t_up + 1.0] or [first]
            sl = [lq[i] for i in steady]
            pick = lambda k: [link[k][i] for i in steady if i < len(link.get(k, []))]
            m = _med(sl)
            drops = sum(1 for q in sl if q < 50)
            d = (f"up after {t_up:.1f} s; then uplink LQ {m:.0f}%, SNR {_med(pick('up_snr_db')):.0f} dB, RSSI "
                 f"{_med(pick('up_rssi1_dbm')):.0f} dBm, downlink LQ {_med(pick('down_lq')):.0f}%, rf_mode "
                 f"{link.get('rf_mode', ['?'])[-1]}, tx power index {link.get('tx_power_idx', ['?'])[-1]}")
            if drops:
                d += f"; {drops} dropout sample(s) after connecting"
            status = PASS if m >= 90 and not drops else WARN if m >= 70 else FAIL
            if status == PASS and t_up > 12:
                status, d = WARN, d + " (slow to connect: receiver RF-mode search, normal after a restart)"
            add(status, "radio link to the receiver", d)

    counts = s["counts"]
    missing = [k for k in FLIGHT_TELEMETRY if not counts.get(k)]
    if not missing:
        b, a = s["last"].get("battery", {}), s["last"].get("attitude", {})
        add(PASS, "flight telemetry (battery, attitude, mode)",
            f"{b.get('voltage_v', 0):.1f} V, pitch {math.degrees(a.get('pitch_rad', 0)):+.0f} roll "
            f"{math.degrees(a.get('roll_rad', 0)):+.0f}, mode {s['last'].get('flight_mode', {}).get('mode')}")
    else:
        add(WARN, "flight telemetry (battery, attitude, mode)", "missing: " + ", ".join(missing) +
            " (enable these sensors in Betaflight telemetry; needs a linked receiver)")

    o = s["last"].get("optrange")
    if counts.get("optrange"):
        rate_o = counts["optrange"] / dur
        add(PASS, "range + flow frame (0xD3)", f"{rate_o:.0f} Hz, distance {o['distance_m']:.3f} m, strength "
                                              f"{o['dist_strength']}, flow quality {o['flow_quality']}")
    else:
        add(WARN, "range + flow frame (0xD3)", "not received: the FC must send it (patched Betaflight) for the sensor watchdog")

    ot = s.get("optrange_t")
    if ot and len(ot) >= 5:
        gaps = [b - a for a, b in zip(ot, ot[1:])]
        mx, over = max(gaps), sum(g > 0.5 for g in gaps)
        d = f"max gap {mx * 1000:.0f} ms, {over} gap(s) over 0.5 s in {len(ot)} frames"
        if mx <= 0.35:
            add(PASS, "0xD3 steady enough for the 0.5 s sensor watchdog", d)
        elif mx <= 0.5:
            add(WARN, "0xD3 steady enough for the 0.5 s sensor watchdog", d + " (marginal)")
        else:
            add(FAIL, "0xD3 steady enough for the 0.5 s sensor watchdog", d + ": the watchdog would false-trip; raise the ELRS "
                                                                           "Telem Ratio (1:2) or send fewer FC sensors")

    found = [k for k in NIMBUS_ONLY if counts.get(k)]
    absent = [k for k in NIMBUS_ONLY if not counts.get(k)]
    add(INFO, "Nimbus-specific frames", ("seen: " + ", ".join(found) if found else "none seen") +
        ("; not sent by this FC/dongle: " + ", ".join(absent) if absent else ""))

    msp = s.get("msp_board")
    if msp is not None:
        add(PASS if msp else WARN, "MSP over CRSF (FC board info)", msp if msp else
            "no reply: FC does not answer MSP through the receiver (stock ELRS passes it only if the FC has MSP telemetry on the RX UART)")

    fc = s.get("fc")
    if fc is not None:
        if not fc.get("reachable"):
            add(FAIL, "flight controller reachable (USB MSP)", "no MSP reply on the FC port")
        else:
            add(PASS if not fc.get("armed") else FAIL, "FC disarmed", "disarmed" if not fc.get("armed") else "FC reports ARMED")
            mask = fc.get("arming_mask")
            if mask is None:
                add(WARN, "FC arming blockers", "could not read arming-disable flags")
            else:
                names = [ARMING_FLAGS[i] for i in range(len(ARMING_FLAGS)) if mask >> i & 1]
                add(PASS if not names else WARN, "FC arming blockers (arm switch low)", "none" if not names else ", ".join(names))
            spans = fc.get("spans")
            if not spans:
                add(FAIL, "channels reach the FC", "no RC data from the FC")
            else:
                rpy = [spans[i][1] - spans[i][0] for i in range(3)]
                thr_max, arm_max = spans[3][1], spans[4][1]
                d = f"roll/pitch/yaw sweep {rpy[0]}/{rpy[1]}/{rpy[2]} us, throttle max {thr_max}, arm channel max {arm_max}"
                if min(rpy) >= 300 and thr_max <= 1100 and arm_max < 1300:
                    add(PASS, "channels reach the FC", d)
                elif min(rpy) < 100 or thr_max > 1100 or arm_max >= 1300:
                    add(FAIL, "channels reach the FC", d + " (FC in failsafe? not receiving, or arm/throttle not low)")
                else:
                    add(WARN, "channels reach the FC", d)
    return out


def verdict(checks: list[Check]) -> int:
    return 1 if any(c.status == FAIL for c in checks) else 0


# ----------------------------------------------------------------------------- collection
def run(port: str, fc_port: Optional[str] = None, duration: float = 25.0, hz: float = 100.0, msp: bool = True,
        log: Callable[[str], None] = lambda m: None) -> dict:
    s = {"duration": duration, "hz": hz, "counts": {}, "last": {}, "link": {}}
    fc = None
    if fc_port:
        fc = FcMsp(fc_port)
        armed = fc.armed()
        if armed:
            fc.close()
            s["fc"] = {"reachable": True, "armed": True}
            s["aborted"] = "FC is armed: disarm it before running doctor"
            s.update(frames_sent=0, max_interval_s=0.0, telemetry_frames=0, crc_errors=0)
            return s
        s["fc"] = {"reachable": armed is not None, "armed": bool(armed)}

    t0 = time.monotonic()

    def on_tel(name: str, values: dict) -> None:
        s["counts"][name] = s["counts"].get(name, 0) + 1
        s["last"][name] = values
        if name == "device_info" and values.get("origin") == 0xEE:
            s["device_info"] = values
        elif name == "device_info" and "device_info" not in s:
            s["device_info"] = values
        if name == "link" and values["up_lq"] >= 50 and "up_at" not in s:
            s["up_at"] = time.monotonic() - t0
        if name == "optrange" and "up_at" in s and time.monotonic() - t0 > s["up_at"] + 3.0:     # only after the link has settled for 3 s
            s.setdefault("optrange_t", []).append(time.monotonic() - t0)
        if name == "link":
            s["link"].setdefault("t", []).append(time.monotonic() - t0)
            for k in ("up_lq", "up_snr_db", "up_rssi1_dbm", "down_lq", "rf_mode", "tx_power_idx"):
                s["link"].setdefault(k, []).append(values[k])

    sess = Session(port, hz=hz, allow_arm=False, ping=True, on_telemetry=on_tel)
    sess.start()
    spans = [[99999, 0] for _ in range(8)]
    try:
        did_msp, did_flags = False, False
        while time.monotonic() - t0 < duration:
            t = time.monotonic() - t0
            sess.set_controls(Controls(roll=0.5 * math.sin(3.0 * t), pitch=0.5 * math.sin(2.3 * t),
                                       yaw=0.5 * math.sin(1.7 * t), arm=False))
            if msp and not did_msp and t > duration * 0.4:
                did_msp = True
                reply = sess.request_msp(commands.MSP_BOARD_INFO, timeout=2.0)
                if reply:
                    try:
                        bi = commands.decode_board_info(reply["payload"])
                        s["msp_board"] = str(bi)
                    except Exception:
                        s["msp_board"] = "reply received"
                else:
                    s["msp_board"] = ""
            if fc and s["fc"]["reachable"]:
                if t > 3:   # let the link and the FC settle
                    rc = fc.rc()
                    if rc and len(rc) >= 5:
                        for i in range(5):
                            spans[i][0] = min(spans[i][0], rc[i]); spans[i][1] = max(spans[i][1], rc[i])
                if not did_flags and t > duration * 0.6:
                    did_flags = True
                    s["fc"]["arming_mask"] = fc.arming_disable_mask()
            else:
                time.sleep(0.02)
            time.sleep(0.01)
        if fc and s["fc"]["reachable"]:
            s["fc"]["armed"] = bool(fc.armed())
            if spans[0][0] != 99999:
                s["fc"]["spans"] = [tuple(x) for x in spans[:5]]
    finally:
        sess.stop()
        if fc:
            fc.close()
    s.update(frames_sent=sess.sender.frames_sent, max_interval_s=sess.sender.max_interval_s,
             telemetry_frames=sess.reader.frame_count, crc_errors=sess.reader.parser.crc_errors)
    return s


def report(checks: list[Check], as_json: bool = False) -> str:
    if as_json:
        return json.dumps([c.__dict__ for c in checks], indent=2)
    width = max(len(c.name) for c in checks)
    lines = [f"[{c.status}] {c.name.ljust(width)}  {c.detail}" for c in checks]
    n = {k: sum(c.status == k for c in checks) for k in (PASS, WARN, FAIL)}
    lines.append(f"\nresult: {n[PASS]} pass, {n[WARN]} warn, {n[FAIL]} fail -> " + ("OK" if not n[FAIL] else "NOT OK"))
    return "\n".join(lines)
