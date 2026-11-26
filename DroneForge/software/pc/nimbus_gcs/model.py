"""GCS model: owns the Session, the controls source and the history for the charts. No Qt imports, so it is testable headless."""
from __future__ import annotations

import glob
import math
import time
from collections import deque
from typing import Optional

from nimbus_pc import live
from nimbus_core import topics as T
from nimbus_core.core import NimbusCore
from nimbus_pc.sender import Controls
from nimbus_pc.watchdog import NAMES as WATCHDOG_NAMES

HISTORY_S = 90.0
SOURCES = ("idle", "sweep", "gamepad", "external")                      # external = the hover autonomy (set by the Hover panel buttons)
SERIES = ("up_lq", "down_lq", "rssi", "battery_v", "range_m", "flow_q", "pitch", "roll",
          "fz_range", "fz_ekf", "fv_flow", "fv_ekf", "fvz_ekf")        # the last five: only FRESH sensor/EKF samples (the SENSORS tab fusion check)


def list_ports() -> list[str]:
    return sorted(glob.glob("/dev/serial/by-id/*")) or sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))


def find_fc_port() -> str:
    return next((p for p in sorted(glob.glob("/dev/serial/by-id/*")) if "Betaflight" in p), "")


def list_gamepads() -> list[tuple[str, str]]:
    """(path, name) of real joysticks; skips virtual devices such as the 'fake mouse' driver."""
    out = []
    for path in sorted(glob.glob("/dev/input/js*")):
        try:
            with open(f"/sys/class/input/{path.rsplit('/', 1)[-1]}/device/name") as f:
                name = f.read().strip()
        except OSError:
            name = "?"
        if "mouse" not in name.lower() and "keyboard" not in name.lower():
            out.append((path, name))
    return out


# LiPo per-cell voltage -> state of charge (%), resting-ish curve. Under load the voltage sags, so the number reads low while flying.
_LIPO = [(3.30, 0), (3.50, 5), (3.60, 10), (3.70, 25), (3.75, 35), (3.80, 50), (3.85, 60), (3.90, 70), (3.95, 78), (4.00, 85), (4.10, 95), (4.20, 100)]


def display_vbat(v, scale: float = 1.0):
    """Pack voltage for display: the FC reading x the user's voltage correction factor (battery_scale, default 1.0). Display only."""
    return None if v is None else v * scale


def battery_percent(pack_v: float, cells: int = 4) -> float:
    """State of charge 0..100 from the pack voltage (piecewise-linear LiPo curve)."""
    v = pack_v / cells
    if v <= _LIPO[0][0]:
        return 0.0
    if v >= _LIPO[-1][0]:
        return 100.0
    for (v0, p0), (v1, p1) in zip(_LIPO, _LIPO[1:]):
        if v <= v1:
            return p0 + (p1 - p0) * (v - v0) / (v1 - v0)
    return 100.0


def warnings(s: dict, frames_in: int, seconds_connected: float) -> list[str]:
    """Banner texts for the GCS (pure, tested)."""
    out = []
    if not s["connected"]:
        return out
    if seconds_connected > 3 and s["no_data_s"] > 3:
        out.append(f"NO DATA from the dongle for {s['no_data_s']:.0f} s: serial port dropped (hub reset?) or the TX is silent")
    if not s["sender_alive"]:
        out.append("SENDER STOPPED: serial write failed. Nothing is being sent to the drone")
    if s["watchdog"] not in ("ok", "off"):
        out.append(f"SENSOR WATCHDOG TRIPPED ({s['watchdog']}): throttle forced to minimum until the arm switch goes off")
    if s.get("tx_stuck"):
        out.append("TX NOT FORWARDING: only link stats arrive (no radio-sync frames, no ping replies, no FC telemetry) while the radio link is up. "
                   "Power-cycle the TX board. Commands still reach the drone; the PC just cannot see its data")
    if s.get("safety_disarm"):
        out.append(f"SAFETY DISARM ({s['safety_disarm']}): arm is forced LOW and throttle to minimum until the arm switch is turned off")
    if s.get("safety_land"):
        out.append(f"SAFETY: LAND requested ({s['safety_land']}): an autonomous controller must land now; a manual pilot should land")
    crc = s["crc_errors"]
    if crc >= 10 and crc > 0.05 * max(frames_in, 1):
        out.append(f"SERIAL ERRORS: {crc} corrupted frames vs {frames_in} good: check the TX UART wires (GPIO20/21/GND), solder joints and the TX power")
    return out


class GcsModel:
    """UI-facing model: a CLIENT of nimbus_core. It never opens the port; it subscribes to the core's topics (live tables, events)
    and forwards commands. Everything the widgets read (state, status, arming checklist, history) is derived here."""

    def __init__(self, core: Optional[NimbusCore] = None) -> None:
        self.core = core or NimbusCore()
        self.state = live.LiveState()
        self.history: dict[str, deque] = {k: deque() for k in SERIES}
        self._t0 = time.monotonic()
        self._vbat_f: Optional[float] = None
        self._vb_hist: deque = deque()                       # (time, volts) of each NEW battery frame, last 10 s
        self.trail: deque = deque(maxlen=900)
        bus = self.core.bus
        bus.subscribe(T.TELEMETRY, lambda m: self.state.note_decoded(*m))
        bus.subscribe(T.FRAME_IN, lambda f: self.state.note_in(f))
        bus.subscribe(T.FRAME_OUT, lambda d: self.state.note_out(d))
        bus.subscribe(T.EVENT, lambda text: self.state.event(text))

    # --- read-through of the core's state (kept for the widgets and the tests) ----------------
    sess = property(lambda self: self.core.sess)
    port = property(lambda self: self.core.port)
    source = property(lambda self: self.core.source)
    allow_arm = property(lambda self: self.core.allow_arm)
    pad = property(lambda self: self.core.pad)
    last_controls = property(lambda self: self.core.last_controls)
    connected = property(lambda self: self.core.connected)
    gamepad_path = property(lambda self: self.core.gamepad_path)

    @property
    def watch_optrange(self) -> bool:
        return self.core.watch_optrange

    @watch_optrange.setter
    def watch_optrange(self, v: bool) -> None:
        self.core.watch_optrange = v

    # --- commands: forwarded to the core -----------------------------------------------------
    def connect(self, port: str, hz: float = 100.0, log_path: Optional[str] = None) -> None:
        self.core.disconnect()
        self.state = live.LiveState()
        self._t0 = time.monotonic()
        for q in self.history.values():
            q.clear()
        self.core.connect(port, hz=hz, log_path=log_path)

    def connect_sim(self) -> None:
        self.core.disconnect()
        self.state = live.LiveState()
        self._t0 = time.monotonic()
        for q in self.history.values():
            q.clear()
        self.core.connect_sim()

    def connect_bench(self, port: str, fc_port: str) -> None:
        """BENCH MODE (props OFF): real FC + radio link, sensors simulated from the FC's motor outputs."""
        self.core.disconnect()
        self.state = live.LiveState()
        self._t0 = time.monotonic()
        for q in self.history.values():
            q.clear()
        self.core.connect(port, bench_fc_port=fc_port)

    @property
    def is_bench(self) -> bool:
        return self.connected and self.core.state_source == "bench"

    @property
    def is_sim(self) -> bool:
        return self.connected and self.core.state_source == "sim"

    def disconnect(self) -> None:
        self.core.disconnect()

    def set_source(self, name: str, gamepad_path: str = "") -> None:
        self.core.set_source(name, gamepad_path)

    def set_allow_arm(self, allow: bool) -> None:
        self.core.set_allow_arm(allow)

    def safe_now(self) -> None:
        self.core.safe_now()

    def tick(self) -> None:
        self.core.tick()

    # --- arming checklist --------------------------------------------------------------------
    def arming_status(self) -> list[tuple[str, Optional[bool], str]]:
        """What must be true for the controller to arm the drone, and what is missing: (item, ok / None = unknown, detail)."""
        d = self.state.decoded
        items: list = []
        items.append(("Connected to the dongle", self.connected, "" if self.connected else "press Connect"))
        if self.source == "external":
            items.append(("Controls source = external (hover autonomy)", True, ""))
        items.append(("Arming allowed", self.allow_arm, "" if self.allow_arm else "tick 'Allow arming' (props off!)"))
        fm = d.get("flight_mode", {}).get("mode")
        if fm:
            items.append(("FC armed (from its flight mode)", not fm.endswith("*"), f"mode {fm} ('*' = disarmed)"))
        else:
            items.append(("FC armed (from its flight mode)", None, "no flight-mode telemetry yet"))
        return items

    def flight_view(self) -> dict:
        """What the Flight view shows: the simulator's ground truth in Sim, the controller's height estimate on the live link."""
        au = self.core.autonomy
        ref = au.cfg.hover_height if au.phase == "flying" else None
        if self.is_bench:
            sim = self.core.bench.sim
            return {"z": sim.z, "x": sim.x, "y": sim.y, "z_ref": ref, "pitch": math.degrees(sim.tilt_x), "roll": math.degrees(sim.tilt_y),
                    "armed": self.core.bench.fc_armed_real, "label": "BENCH: real FC + link, simulated drone body (props OFF)", "truth": True}
        if self.is_sim:
            sim = self.core.sess.sim
            return {"z": sim.z, "x": sim.x, "y": sim.y, "z_ref": ref, "pitch": math.degrees(sim.tilt_x), "roll": math.degrees(sim.tilt_y),
                    "armed": self.core.sess.fc_armed, "label": "SIMULATOR ground truth" + ("  CRASHED" if sim.crashed else ""), "truth": True}
        att = self.state.decoded.get("attitude")
        mode = (self.state.decoded.get("flight_mode") or {}).get("mode") or ""
        return {"z": au.ctl.z if self.connected else 0.0, "x": None, "y": None, "z_ref": ref,
                "pitch": math.degrees(att["pitch_rad"]) if att else 0.0, "roll": math.degrees(att["roll_rad"]) if att else 0.0,
                "armed": bool(mode) and not mode.endswith("*"), "label": "live: height = controller estimate (range sensor), no position source", "truth": False}

    def arming_summary(self) -> str:
        bad = [f"{n}: {dtl}" if dtl else n for n, ok, dtl in self.arming_status()[:8] if ok is False]
        if self.source == "external":
            return "Hover autonomy is in control: use 'Take off + hold' in the Hover panel." if not bad else "Missing -> " + bad[0]
        return "Ready: arm from the Fly page." if not bad else "Missing -> " + bad[0]

    # --- snapshot for the UI ----------------------------------------------------------------
    def sample(self) -> None:
        """Append the current values to the chart history (called ~10 Hz)."""
        t = time.monotonic() - self._t0
        d = self.state.decoded
        v = d.get("battery", {}).get("voltage_v")
        if not self.connected:
            self._vbat_f = None; self._vb_hist.clear()
        elif v is not None:                                 # median of the new frames of the last 10 s: the FC reading jumps 0.4-0.9 V inside 10 s even at rest (2026-11-26 recording), the old 2 s smoothing let that through
            now = time.monotonic()
            if not self._vb_hist or v != self._vb_hist[-1][1] or now - self._vb_hist[-1][0] > 1.5: self._vb_hist.append((now, v))
            while self._vb_hist and now - self._vb_hist[0][0] > 10.0: self._vb_hist.popleft()
            vals = sorted(x[1] for x in self._vb_hist); self._vbat_f = vals[len(vals) // 2]
        vals = {
            "up_lq": d.get("link", {}).get("up_lq"), "down_lq": d.get("link", {}).get("down_lq"),
            "rssi": d.get("link", {}).get("up_rssi1_dbm"), "battery_v": display_vbat(d.get("battery", {}).get("voltage_v"), self._vscale()),
            "range_m": d.get("optrange", {}).get("distance_m"), "flow_q": d.get("optrange", {}).get("flow_quality"),
            "pitch": math.degrees(d["attitude"]["pitch_rad"]) if "attitude" in d else None,
            "roll": math.degrees(d["attitude"]["roll_rad"]) if "attitude" in d else None,
        }
        ao, ak = self.state.age("optrange"), self.state.age("kinematic")
        o, kin = d.get("optrange"), d.get("kinematic")
        if self.connected and o and ao is not None and ao < 1.0 and o.get("distance_m", 0) > 0:
            vals["fz_range"] = o["distance_m"]; vals["fv_flow"] = math.hypot(o["velocity_forward"], o["velocity_right"])
        if self.connected and kin and ak is not None and ak < 1.0 and kin.get("valid"):
            vals["fz_ekf"] = kin["z_m"]; vals["fvz_ekf"] = kin["vz_mps"]; vals["fv_ekf"] = math.hypot(kin["vx_mps"], kin["vy_mps"])
        if self.is_sim or self.is_bench:
            sim = self.core.sess.sim if self.is_sim else self.core.bench.sim
            self.trail.append((sim.x, sim.y))
        elif not self.connected:
            self.trail.clear()
        for k, v in vals.items():
            q = self.history[k]
            if v is not None:
                q.append((t, float(v)))
            while q and t - q[0][0] > HISTORY_S:
                q.popleft()

    def channels_us(self) -> list[int]:
        if not self.sess:
            return [1500, 1500, 988, 1500, 988, 988, 988, 1500]
        ch = self.last_controls.to_channels(self.sess.sender.allow_arm)
        return [int(round(988 + (v - 172) * (2012 - 988) / (1811 - 172))) for v in ch[:8]]

    def _vscale(self) -> float:
        try:
            return float(self.core.autonomy.settings.battery_scale)
        except (AttributeError, TypeError, ValueError):
            return 1.0

    def raw_vbat(self) -> Optional[float]:
        """Smoothed RAW pack voltage as the FC reports it (no correction), for calibration."""
        if not self.connected:
            return None
        if self._vbat_f is not None:
            return self._vbat_f
        return self.state.decoded.get("battery", {}).get("voltage_v")

    def _vbat(self) -> Optional[float]:
        """Smoothed pack voltage; falls back to the latest raw reading before the first smoothing step."""
        if not self.connected:
            return None
        if self._vbat_f is not None:
            return display_vbat(self._vbat_f, self._vscale())
        return display_vbat(self.state.decoded.get("battery", {}).get("voltage_v"), self._vscale()) if self.connected else None      # the same correction as the shown voltage

    def status(self) -> dict:
        s, st = self.sess, self.state
        d = st.decoded
        link = d.get("link", {})
        now = st.clock()
        lasts = [q[-1] for q in st.in_times.values() if q]
        others = [q[-1] for t, q in st.in_times.items() if q and t != 0x14]
        link_age = (now - st.in_times[0x14][-1]) if st.in_times.get(0x14) else None
        others_age = (now - max(others)) if others else (now - st.t0)
        tx_stuck = bool(s and (now - st.t0) > 8.0 and link_age is not None and link_age < 1.5 and others_age > 4.0)
        return {
            "tx_stuck": tx_stuck,
            "safety_disarm": self.core.safety.disarm_latched, "safety_land": self.core.safety.land_request,
            "sim": self.is_sim, "bench": self.is_bench,
            "connected": self.connected, "port": self.port, "source": self.source, "allow_arm": self.allow_arm,
            "fps": (s.sender.frames_sent / max(now - st.t0, 0.5)) if s else 0.0,
            "max_gap_ms": s.sender.max_interval_s * 1000 if s else 0.0,
            "crc_errors": s.reader.parser.crc_errors if s else 0,
            "sender_alive": bool(s and s.sender.alive),
            "no_data_s": (now - max(lasts)) if lasts else ((now - st.t0) if s else 0.0),
            "lq": link.get("up_lq"), "rssi": link.get("up_rssi1_dbm"), "snr": link.get("up_snr_db"),
            "battery_v": self._vbat(),                                     # the same steady value as the percentage
            "battery_pct": None if self._vbat() is None else battery_percent(self._vbat(), self.core.safety.cfg.cells),
            "battery_cell_v": None if self._vbat() is None else self._vbat() / self.core.safety.cfg.cells, "mode": d.get("flight_mode", {}).get("mode"),
            "failsafe": bool(s and s.sender.failsafe_active),
            "watchdog": WATCHDOG_NAMES[s.watchdog.state] if (s and s.watchdog) else "off",
            "arm_high": bool(s and self.last_controls.arm and s.sender.allow_arm),
            "rf_timing": live.rf_timing(st),
        }
