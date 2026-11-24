"""`nimbus_pc live`: real-time terminal view of everything that flows over the dongle link, in both directions.

Layout follows what NimbusOS's core publishes to its UI (dist/schema: TelemetryMessage groups attitude, baroVario, battery,
droneLinked, flightControllerUid, flightMode, gps, linkStats, magnetometer, mightyCam, motorRpm, optrange, rangefinder, rawImu,
vario, videoReceiverTopChannels; ControlsMessage with its source; DeviceInfoMessage), so the two can be compared side by side.
Pure rendering (`render`) is separated from the serial loop so it can be tested without hardware.
"""
from __future__ import annotations

import math
import sys
import time
from collections import deque
from typing import Callable, Optional

from .crsf import Frame, FrameParser
from .watchdog import NAMES as WATCHDOG_NAMES

# CRSF frame type -> (Nimbus TelemetryMessage group, short label)
TYPE_INFO = {
    0x02: ("gps", "GPS"), 0x07: ("vario", "VARIO"), 0x08: ("battery", "BATTERY"), 0x09: ("baroVario", "BARO_ALT"),
    0x12: ("magnetometer", "MAG"), 0x14: ("linkStats", "LINK_STATS"), 0x16: ("", "RC_CHANNELS"), 0x1E: ("attitude", "ATTITUDE"),
    0x21: ("flightMode", "FLIGHT_MODE"), 0x28: ("", "DEVICE_PING"), 0x29: ("", "DEVICE_INFO"), 0x2B: ("", "PARAM_ENTRY"),
    0x2C: ("", "PARAM_READ"), 0x2D: ("", "PARAM_WRITE"), 0x32: ("", "COMMAND"), 0x7A: ("", "MSP_REQ"), 0x7B: ("", "MSP_RESP"),
    0x7C: ("", "MSP_WRITE"), 0xD0: ("rangefinder", "RANGEFINDER*"), 0xD1: ("rawImu", "RAW_IMU*"), 0xD2: ("mightyCam", "MIGHTY_CAM*"),
    0xD3: ("optrange", "OPTRANGE*"), 0xD4: ("kinematic", "KINEMATIC*"), 0xD5: ("motorRpm", "MOTOR_ERPM*"),
    0xE1: ("videoReceiverTopChannels", "VRX_TOP*"), 0xE2: ("droneLinked", "DRONE_LINKED*"), 0xE3: ("flightControllerUid", "FC_UID*"),
    0xE4: ("", "CUSTOM_E4*"), 0x3A: ("", "RADIO_ID_SYNC"),
}
# telemetry groups in the order NimbusOS's TelemetryMessage lists them
GROUPS = ["attitude", "baroVario", "battery", "droneLinked", "flightControllerUid", "flightMode", "gps", "linkStats",
          "magnetometer", "kinematic", "mightyCam", "motorRpm", "optrange", "rangefinder", "rawImu", "vario", "videoReceiverTopChannels"]
# group -> decoded name(s) produced by crsf.decode_telemetry
DECODED = {"attitude": "attitude", "baroVario": "baro", "battery": "battery", "droneLinked": "drone_linked",
           "flightControllerUid": "fc_uid", "flightMode": "flight_mode", "linkStats": "link", "magnetometer": "magnetometer",
           "kinematic": "kinematic", "mightyCam": "custom_d2", "optrange": "optrange", "rangefinder": "rangefinder", "rawImu": "raw_imu", "vario": "vario",
           "videoReceiverTopChannels": "vrx_top_channels"}
GROUP_TYPES = {g: [t for t, (grp, _) in TYPE_INFO.items() if grp == g] for g in GROUPS}
RATE_WINDOW = 5.0


class LiveState:
    def __init__(self, clock: Callable[[], float] = time.monotonic):
        self.clock = clock
        self.t0 = clock()
        self.in_times: dict[int, deque] = {}
        self.out_times: dict[int, deque] = {}
        self.in_count: dict[int, int] = {}
        self.out_count: dict[int, int] = {}
        self.last_payload: dict[int, bytes] = {}
        self.decoded: dict[str, dict] = {}
        self.decoded_t: dict[str, float] = {}
        self.events: deque = deque(maxlen=40)
        self.raw: deque = deque(maxlen=40)
        self.device_info: dict[int, dict] = {}
        self._out_parser = FrameParser()
        self._prev: dict[str, object] = {}

    # --- inputs -------------------------------------------------------------------------------
    def event(self, text: str) -> None:
        self.events.append((self.clock() - self.t0, text))

    def _tick(self, table: dict, counts: dict, ftype: int) -> None:
        now = self.clock()
        q = table.setdefault(ftype, deque())
        q.append(now)
        while q and now - q[0] > RATE_WINDOW:
            q.popleft()
        counts[ftype] = counts.get(ftype, 0) + 1

    def note_in(self, frame: Frame) -> None:
        self._tick(self.in_times, self.in_count, frame.type)
        self.last_payload[frame.type] = bytes(frame.payload)
        self.raw.append((self.clock() - self.t0, "<-", frame.type, bytes(frame.payload)))

    def note_out(self, data: bytes) -> None:
        for f in self._out_parser.feed(data):
            self._tick(self.out_times, self.out_count, f.type)
            if f.type != 0x16:                               # RC frames would flood the raw list; they have their own counter
                self.raw.append((self.clock() - self.t0, "->", f.type, bytes(f.payload)))

    def note_decoded(self, name: str, values: dict) -> None:
        self.decoded[name] = values
        self.decoded_t[name] = self.clock()
        if name == "device_info":
            self.device_info[values.get("origin", 0)] = values
        # change events worth a log line
        key = {"flight_mode": ("mode", "flight mode"), "drone_linked": ("is_linked", "drone linked")}.get(name)
        if key and self._prev.get(name) != values.get(key[0]):
            if name in self._prev:
                self.event(f"{key[1]} -> {values.get(key[0])}")
            self._prev[name] = values.get(key[0])
        if name == "link":
            up = values["up_lq"] >= 50
            if self._prev.get("link_up") != up:
                if "link_up" in self._prev:
                    self.event("radio link " + ("UP" if up else "LOST"))
                self._prev["link_up"] = up

    # --- derived ------------------------------------------------------------------------------
    def rate(self, table: dict, ftype: int) -> float:
        q = table.get(ftype)
        if not q:
            return 0.0
        now = self.clock()
        for _ in range(5):                                   # the radio thread appends to this deque while the GUI counts: count a snapshot
            try:
                snap = tuple(q)
                break
            except RuntimeError:
                continue
        else:
            return 0.0
        n = sum(1 for x in snap if now - x <= RATE_WINDOW)
        return n / min(RATE_WINDOW, max(now - self.t0, 0.5))

    def age(self, name: str) -> Optional[float]:
        t = self.decoded_t.get(name)
        return None if t is None else self.clock() - t


# ------------------------------------------------------------------------------------------------
RESET, DIM, RED, GREEN, YELLOW, CYAN, BOLD = "\x1b[0m", "\x1b[2m", "\x1b[31m", "\x1b[32m", "\x1b[33m", "\x1b[36m", "\x1b[1m"


def rf_timing(st: LiveState) -> Optional[str]:
    """CRSF RADIO_ID (0x3A) sub-type 0x10: the TX tells the handset the radio packet interval and phase offset (0.1 us units)."""
    p = st.last_payload.get(0x3A)
    if not p or len(p) < 11 or p[2] != 0x10:
        return None
    interval = int.from_bytes(p[3:7], "big") / 10.0
    offset = int.from_bytes(p[7:11], "big", signed=True) / 10.0
    return f"radio packet interval {interval:.0f} us = {1e6 / interval:.0f} Hz, phase offset {offset:+.0f} us (0x3A sync frames {st.rate(st.in_times, 0x3A):.0f}/s)" if interval else None


def _age_txt(age: Optional[float], color: bool) -> str:
    if age is None:
        return f"{'--':>6}"
    s = f"{age:5.1f}s"
    if not color:
        return s
    return (GREEN if age < 1.0 else YELLOW if age < 3.0 else RED) + s + RESET


def summarize(group: str, st: LiveState) -> str:
    """One-line value text for a Nimbus telemetry group; raw hex for frames we cannot decode."""
    v = st.decoded.get(DECODED.get(group, ""), None)
    if v is not None:
        if group == "kinematic":
            return (f"pos ({v['x_m']:+.2f},{v['y_m']:+.2f},{v['z_m']:+.2f}) m  vel ({v['vx_mps']:+.2f},{v['vy_mps']:+.2f},{v['vz_mps']:+.2f}) m/s" + ("" if v["valid"] else "  [NOT INITIALISED]"))
        if group == "attitude":
            return f"pitch {math.degrees(v['pitch_rad']):+6.1f}  roll {math.degrees(v['roll_rad']):+6.1f}  yaw {math.degrees(v['yaw_rad']):+6.1f} deg"
        if group == "battery":
            return f"{v['voltage_v']:.2f} V  {v['current_a']:.1f} A  {v['capacity_mah']} mAh  {v['remaining_pct']}%"
        if group == "baroVario":
            return f"baro altitude {v['baro_alt_m']:.2f} m"
        if group == "vario":
            return f"vertical speed {v['vspeed_cm_s']} cm/s"
        if group == "flightMode":
            return str(v["mode"])
        if group == "linkStats":
            return (f"up LQ {v['up_lq']}% RSSI {v['up_rssi1_dbm']}/{v['up_rssi2_dbm']} dBm SNR {v['up_snr_db']} | down LQ {v['down_lq']}% "
                    f"RSSI {v['down_rssi_dbm']} | mode {v['rf_mode']} pwr {v['tx_power_idx']}")
        if group == "optrange":
            return (f"dist {v['distance_m']:.3f} m str {v['dist_strength']} | flow q {v['flow_quality']} "
                    f"xy ({v['flow_x_raw']},{v['flow_y_raw']}) vel fwd {v['velocity_forward']:+.2f} right {v['velocity_right']:+.2f} m/s")
        if group == "droneLinked":
            return "linked" if v.get("is_linked") else "NOT linked"
        if group == "flightControllerUid":
            return str(v.get("uid_hex", v))
        return " ".join(f"{k}={x}" for k, x in list(v.items())[:5] if k != "t")
    for t in GROUP_TYPES.get(group, []):
        if t in st.last_payload:
            return "raw " + st.last_payload[t][:14].hex() + ("..." if len(st.last_payload[t]) > 14 else "") + f" (type 0x{t:02X}, not decoded)"
    return "--"


def render(st: LiveState, *, port: str = "", sess=None, source: str = "", color: bool = True, width: int = 118,
           raw_lines: int = 0) -> str:
    now = st.clock()
    out = []
    add = out.append
    up = int(now - st.t0)
    fps = sess.sender.frames_sent / max(now - st.t0, 0.5) if sess else 0.0
    gap = sess.sender.max_interval_s * 1000 if sess else 0.0
    crc = sess.reader.parser.crc_errors if sess else 0
    nin = sum(st.in_count.values())
    head = f"NIMBUS PC LIVE  {port}  up {up // 60:02d}:{up % 60:02d}   OUT {fps:4.0f} fr/s (max gap {gap:4.1f} ms)   IN {nin} frames  CRC errors {crc}"
    add((BOLD if color else "") + head + (RESET if color else ""))
    warn = []
    lasts = [q[-1] for q in st.in_times.values() if q]
    if now - st.t0 > 3.0 and (not lasts or now - max(lasts) > 3.0):
        warn.append(f"NO DATA from the dongle for {now - (max(lasts) if lasts else st.t0):.0f} s: serial port dropped (hub reset?) or the TX is silent")
    if sess is not None and not sess.listen_only and not sess.sender.alive and now - st.t0 > 1.0:
        warn.append("SENDER STOPPED: serial write failed (port lost). Nothing is being sent to the drone")
    for w in warn:
        add((RED + BOLD if color else "") + "!! " + w + (RESET if color else ""))
    add("-" * width)

    add("TELEMETRY IN  (same groups as the Nimbus TelemetryMessage; * rows = custom frames)")
    add(f"  {'group':<25s} {'value':<73s} {'age':>6s} {'Hz':>5s}")
    for g in GROUPS:
        name = DECODED.get(g)
        age = st.age(name) if name else None
        if age is None:                                        # undecoded: age from the raw frame stream
            ts = [st.in_times[t][-1] for t in GROUP_TYPES.get(g, []) if st.in_times.get(t)]
            age = (now - max(ts)) if ts else None
        hz = sum(st.rate(st.in_times, t) for t in GROUP_TYPES.get(g, []))
        add(f"  {g:<25s} {summarize(g, st):<73.73s} {_age_txt(age, color)} {hz:5.1f}")

    rt = rf_timing(st)
    add("  RF timing: " + (rt if rt else "--"))
    add("")
    add("COMMANDS OUT  (Nimbus ControlsMessage)")
    if sess:
        c = sess.sender.current_controls()
        ch = c.to_channels(sess.sender.allow_arm)
        us = lambda v: int(round(988 + (v - 172) * (2012 - 988) / (1811 - 172)))
        wd = WATCHDOG_NAMES[sess.watchdog.state] if sess.watchdog else "off"
        add(f"  source {source or 'idle':<8s} roll {c.roll:+.2f} pitch {c.pitch:+.2f} yaw {c.yaw:+.2f} throttle {c.throttle:.2f} "
            f"arm {'ON ' if (c.arm and sess.sender.allow_arm) else 'off'}   stale-failsafe {'ON' if sess.sender.failsafe_active else 'off'}  "
            f"sensor-watchdog {wd}")
        add(f"  channels us: " + " ".join(f"{us(v):4d}" for v in ch[:8]) + f"   (allow_arm={sess.sender.allow_arm})")
    di = st.device_info
    names = {0xEE: "TX", 0xEC: "RX", 0xC8: "FC"}
    add("DEVICES  " + ("  ".join(f"{names.get(o, hex(o))}: '{d.get('name', '?')}'" for o, d in sorted(di.items())) or "no DEVICE_INFO yet"))

    add("")
    add("FRAMES        IN: type            count    Hz  age        |  OUT: type          count    Hz")
    ins = sorted(st.in_count.items(), key=lambda kv: -st.rate(st.in_times, kv[0]))[:9]
    outs = sorted(st.out_count.items(), key=lambda kv: -kv[1])[:9]
    for i in range(max(len(ins), len(outs), 1)):
        a = b = ""
        if i < len(ins):
            t, n = ins[i]
            lastt = st.in_times[t][-1] if st.in_times.get(t) else None
            a = f"0x{t:02X} {TYPE_INFO.get(t, ('', '?'))[1]:<15s} {n:6d} {st.rate(st.in_times, t):5.1f} {_age_txt(None if lastt is None else now - lastt, color)}"
        if i < len(outs):
            t, n = outs[i]
            b = f"0x{t:02X} {TYPE_INFO.get(t, ('', '?'))[1]:<14s} {n:6d} {st.rate(st.out_times, t):5.1f}"
        add(f"  {a:<68s}|  {b}")
    add("(* = Nimbus custom frame types)")

    add("")
    add("EVENTS")
    for t, txt in list(st.events)[-6:]:
        add(f"  +{t:7.1f}s  {txt}")
    if not st.events:
        add("  (none yet)")
    if raw_lines:
        add("")
        add("RAW FRAMES (last %d; RC_CHANNELS out are counted, not listed)" % raw_lines)
        for t, d, ft, pl in list(st.raw)[-raw_lines:]:
            add(f"  +{t:7.1f}s {d} 0x{ft:02X} {TYPE_INFO.get(ft, ('', '?'))[1]:<14s} {pl[:20].hex()}{'...' if len(pl) > 20 else ''}")
    return "\n".join(out)


def run(sess, st: LiveState, *, port: str, duration: float, source: str, set_controls: Callable[[], None],
        raw_lines: int = 0, fps: float = 5.0, stream=None, clear: bool = True, color: bool = True) -> str:
    """Draw until `duration` seconds (0 = until Ctrl-C). Returns the last frame (for tests)."""
    stream = stream or sys.stdout
    last = ""
    end = None if duration <= 0 else time.monotonic() + duration
    next_draw = 0.0
    try:
        while end is None or time.monotonic() < end:
            set_controls()                                   # ~50 Hz: keeps the sender's stale-input failsafe quiet
            now = time.monotonic()
            if now >= next_draw:
                last = render(st, port=port, sess=sess, source=source, color=color, raw_lines=raw_lines)
                stream.write(("\x1b[H\x1b[J" if clear else "\n") + last + "\n")
                stream.flush()
                next_draw = now + 1.0 / fps
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    return last
