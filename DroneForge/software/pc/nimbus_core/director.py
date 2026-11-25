"""FlightDirector: the ONLY thing the GUI calls for flying (QGroundControl style: one call per button, nothing is ever asked back).

Every command returns Result(ok, message) and never raises or asks: a refusal is just ok=False + a sentence for an inline status line.
state() is a cheap snapshot for polling; preflight() is the checklist table. Qt-free; the pure check logic is evaluate_preflight(dict)."""
from __future__ import annotations

import re
import time
from dataclasses import dataclass, field, replace
from typing import NamedTuple, Optional

from . import settings as S
from . import topics as T


class Result(NamedTuple):
    ok: bool                       # accepted / issued (unchanged meaning)
    message: str
    status: str = "issued"         # 'issued' | 'confirmed' | 'timed_out' | 'denied'
    command_id: int = 0


@dataclass(frozen=True)
class CommandState:
    id: int
    name: str                      # arm|disarm|takeoff|land|safe|set_height
    status: str                    # issued|confirmed|timed_out|denied
    message: str
    started_t: float               # director clock (time.monotonic)
    deadline_t: float              # == started_t for commands that finish at once


@dataclass(frozen=True)
class Check:
    id: str
    label: str
    ok: bool
    detail: str
    blocking: bool          # True: when ok is False this check refuses arm/take-off (only connection, armed state, a running flight, safety latch); False: warning only
    fix_hint: str = ""      # one human sentence on how to fix it
    goto: Optional[tuple] = None   # (workspace, sub-tab) the GUI opens to fix it; None = nothing to open


@dataclass(frozen=True)
class FlightState:
    phase: str               # disconnected|ready|arming|armed|takeoff|climbing|holding|landing|landed|fault
    armed: bool
    height_m: Optional[float]
    target_m: float
    link_lq: Optional[float]
    battery_v: Optional[float]
    battery_pct: Optional[int]   # None if unknown
    fc_mode: str
    message: str
    warnings: list = field(default_factory=list)
    command: Optional[CommandState] = None      # latest command and its lifecycle
    mission_phase: str = "safe"                 # safe|ready|armed|takeoff|climb|hold|land
    phase_elapsed_s: float = 0.0
    link_state: str = "offline"                 # offline|connecting|connected|degraded|lost (LinkSupervisor)
    flight_mode: str = "pos_hold"               # stabilize|alt_hold|pos_hold (not saved; pos_hold after each flight)


LQ_MIN, TELEM_FRESH_S = 80.0, 1.0
TILT_OK_DEG, TILT_BLOCK_DEG = 0.5, 1.5
GROUND_RANGE_M = (0.02, 0.15)
HINTS = {   # check id -> (fix_hint, goto)
    "connected": ("Connect the radio dongle and press Connect.", ("SETTINGS", "Settings")),
    "link": ("Move closer to the drone or check the transmitter and receiver.", ("SETUP", "Radio")),
    "telemetry": ("Wait a moment; if it stays, check the radio link and the FC power.", ("SETUP", "Radio")),
    "disarmed": ("Press SAFE to disarm the flight controller.", None),
    "rest_tilt": ("Place the drone on a level surface, then SETUP > Calibration > Level.", ("SETUP", "Calibration")),
    "range": ("Check the range sensor is clear and the drone sits on the ground.", ("SETUP", "Sensors")),
    "autonomy_idle": ("Wait for the current flight or test to finish, or press SAFE.", None),
    "safety": ("Flip the arm switch off, or press SAFE, to clear the safety latch.", None),
    "battery": ("Check the battery is connected and the voltage reading is on.", ("SETTINGS", "Settings")),
    "fc_usb": ("Unplug the FC USB cable from this PC before arming.", None),
    "ceiling": ("Lower the hold height or raise the ceiling in Settings.", ("SETTINGS", "Settings")),
}


def evaluate_preflight(st: dict) -> list[Check]:
    """Pure checklist. st keys (None = unknown): connected, simulated, link_lq, telemetry_age_s, fc_armed, rest_tilt_deg, range_m, range_age_s,
    autonomy_idle, safety_latch (str|None), battery_v, fc_usb (bool), height_m (the height setting), ceiling_m."""
    sim = bool(st.get("simulated"))
    out: list[Check] = []
    add = lambda i, lab, ok, det, blocking=True: out.append(Check(i, lab, bool(ok), det, blocking))
    add("connected", "Connected to the drone", st.get("connected"), "connected" if st.get("connected") else "not connected")
    lq = st.get("link_lq")
    add("link", f"Radio link quality >= {LQ_MIN:.0f}", sim or (lq is not None and lq >= LQ_MIN), "simulated" if sim else ("no link data" if lq is None else f"LQ {lq:.0f}"), blocking=False)
    age = st.get("telemetry_age_s")
    add("telemetry", "FC telemetry fresh", sim or (age is not None and age < TELEM_FRESH_S), "simulated" if sim else ("no telemetry" if age is None else f"{age:.1f} s old"), blocking=False)
    add("disarmed", "FC disarmed", not st.get("fc_armed"), "armed" if st.get("fc_armed") else "disarmed")
    tilt = st.get("rest_tilt_deg")
    if sim:
        add("rest_tilt", "Drone level at rest", True, "simulated")
    elif tilt is None:
        add("rest_tilt", "Drone level at rest", False, "no attitude data", blocking=False)
    else:
        add("rest_tilt", "Drone level at rest", tilt < TILT_OK_DEG, f"{tilt:.2f} deg" + ("" if tilt < TILT_OK_DEG else " (re-level the FC)"), blocking=False)
    r = st.get("range_m")
    fresh = st.get("range_age_s") is not None and st["range_age_s"] < TELEM_FRESH_S
    lo, hi = GROUND_RANGE_M
    add("range", "Range sensor reads the ground", sim or (r is not None and fresh and lo <= r <= hi),
        "simulated" if sim else ("no range data" if r is None or not fresh else f"{r:.3f} m" + ("" if lo <= r <= hi else f" (expected {lo}-{hi} m)")), blocking=False)
    add("autonomy_idle", "No flight in progress", st.get("autonomy_idle"), "idle" if st.get("autonomy_idle") else "a flight or test is running")
    latch = st.get("safety_latch")
    add("safety", "No safety latch", not latch, f"latched: {latch}" if latch else "clear")
    bv = st.get("battery_v")
    add("battery", "Battery reading present", bv is not None, "no reading" if bv is None else f"{bv:.1f} V", blocking=False)
    usb = bool(st.get("fc_usb")) and not sim
    add("fc_usb", "FC USB cable unplugged", not usb, "FC USB attached to this PC: unplug it before arming" if usb else "not attached", blocking=False)
    h, c = st.get("height_m"), st.get("ceiling_m")
    add("ceiling", "Height setting within the ceiling", h is not None and c is not None and h <= c, f"{h} m vs ceiling {c} m" + ("" if (h is not None and c is not None and h <= c) else ": take-off will be limited to the ceiling"), blocking=False)
    return [replace(c, fix_hint=HINTS[c.id][0], goto=HINTS[c.id][1]) if c.id in HINTS else c for c in out]


def _check_sig(checks) -> tuple:
    """What counts as a flip: ok/blocking changes or a changed detail (numbers ignored so 'LQ 97' -> 'LQ 98' is not a flip)."""
    return tuple((c.id, c.ok, c.blocking, re.sub(r"[\d.]+", "#", c.detail)) for c in checks)


def _command(name: str):
    def deco(fn):
        def wrapper(self, *a, **k):
            return self._register(name, fn(self, *a, **k))
        wrapper.__name__, wrapper.__doc__ = fn.__name__, fn.__doc__
        return wrapper
    return deco


class FlightDirector:
    ARM_DEADLINE_S, ARM_DEADLINE_SIM_S = 4.0 + 3.0 + 8.0, 6.0      # gyro cal + countdown + margin
    TAKEOFF_EXTRA_S, DISARM_DEADLINE_S, LAND_DEADLINE_S = 10.0, 5.0, 45.0
    _IMMEDIATE = ("safe", "set_height")

    def __init__(self, core) -> None:
        self.core = core
        self.clock = time.monotonic
        self._cmds: dict[int, CommandState] = {}
        self._cmd_seq = 0
        self._safed = False                     # SAFE pressed and nothing started since
        self._mp = ("safe", self.clock())               # (mission phase, since)
        self._pf: Optional[list] = None         # cached checklist (refreshed by housekeeping, ~5 Hz)
        self._pf_t = -1e9
        self._pf_sig = None
        self._batt_low = False
        self._granted = False                   # True while WE enabled arming (never a user setting)
        self._tel = {"att": None, "t_att": None, "range": None, "t_range": None, "batt": None, "lq": None, "t_any": None}
        core.bus.subscribe(T.TELEMETRY, self._on_tel)

    # --- inputs ---------------------------------------------------------------------------------
    def _on_tel(self, msg) -> None:
        import math
        name, d = msg
        now, t = time.monotonic(), self._tel
        if name == "attitude":
            t["att"], t["t_att"], t["t_any"] = (math.degrees(d["pitch_rad"]), math.degrees(d["roll_rad"])), now, now
        elif name == "optrange":
            t["range"], t["t_range"] = d["distance_m"], now
        elif name == "battery":
            t["batt"] = d["voltage_v"]
        elif name == "link":
            t["lq"] = d["up_lq"]

    @property
    def _au(self):
        return self.core.autonomy

    def _sim(self) -> bool:
        return self.core.state_source in ("sim", "bench")

    # --- pre-flight -----------------------------------------------------------------------------
    def preflight_input(self) -> dict:
        now, t, au = time.monotonic(), self._tel, self._au
        att = t["att"]
        fc_usb = False
        try:
            from nimbus_gcs.fc_logs import fc_serial_port
            fc_usb = fc_serial_port() is not None
        except Exception:
            pass
        return dict(connected=self.core.connected, simulated=self._sim(), link_lq=t["lq"], telemetry_age_s=None if t["t_any"] is None else now - t["t_any"],
                    fc_armed=au.fc_armed, rest_tilt_deg=None if att is None or now - t["t_att"] > TELEM_FRESH_S else max(abs(att[0]), abs(att[1])),
                    range_m=t["range"], range_age_s=None if t["t_range"] is None else now - t["t_range"], autonomy_idle=not au.busy,
                    safety_latch=self.core.safety.disarm_latched, battery_v=t["batt"], fc_usb=fc_usb, height_m=au.settings.hover_height, ceiling_m=au.settings.ceiling_m)

    def preflight(self) -> list[Check]:
        cs = evaluate_preflight(self.preflight_input())
        self._pf, self._pf_t = cs, self.clock()
        self._pf_flip(cs)
        return cs

    def preflight_summary(self) -> tuple:
        """(n_ok, n_total, cautions): cautions = failing checks that do not block (warnings)."""
        cs = self._pf if self._pf is not None else self.preflight()
        return sum(c.ok for c in cs), len(cs), sum(1 for c in cs if not c.ok and not c.blocking)

    def _pf_flip(self, cs) -> None:
        """Publish the checklist on T.PREFLIGHT only when a check flips; notify about checks that turned bad / good."""
        sig = _check_sig(cs)
        if sig == self._pf_sig:
            return
        old = {c[0]: c for c in (self._pf_sig or ())}
        self._pf_sig = sig
        self.core.bus.publish(T.PREFLIGHT, list(cs))
        if old and self.core.connected:
            for c in cs:
                o = old.get(c.id)
                if c.id == "connected" or o is None or o[1] == c.ok:
                    continue
                if not c.ok:
                    self.core.notify(f"{c.label}: {c.detail}", "caution", "preflight")
                else:
                    self.core.notify(f"{c.label}: OK", "info", "preflight")

    def _refusal(self, skip=()) -> Optional[str]:
        bad = [c for c in self.preflight() if not c.ok and c.blocking and c.id not in skip]
        return "; ".join(f"{c.label}: {c.detail}" for c in bad) if bad else None

    # --- commands -------------------------------------------------------------------------------
    def _grant(self) -> None:
        if self.core.source != "external":
            self.core.set_source("external")
        if not self.core.allow_arm:
            self.core.set_allow_arm(True); self._granted = True

    def _revoke(self) -> None:
        if self._granted:
            self._granted = False
            if self.core.allow_arm:
                self.core.set_allow_arm(False)

    def housekeeping(self) -> None:
        """Called from the core tick: drop the arming permission once nothing is running any more; update command lifecycle, checklist and phase."""
        if self._granted and not self._au.busy:
            self._revoke()
        now = self.clock()
        self._update_commands(now)
        if now - self._pf_t >= 0.2:
            self.preflight()
        bv, cfg = self._tel["batt"], self.core.safety.cfg
        low = bv is not None and bv / cfg.cells < cfg.batt_warn_v_cell
        if low != self._batt_low:
            self._batt_low = low
            if low:
                self.core.notify(f"Battery reads {bv:.1f} V (warning only)", "caution", "battery")
        self._mission_phase(now)

    # --- command lifecycle ----------------------------------------------------------------------
    def _register(self, name: str, res: Result) -> Result:
        now = self.clock()
        self._cmd_seq += 1
        cid = self._cmd_seq
        if not res.ok:
            status, dl = "denied", now
        elif name in self._IMMEDIATE:
            status, dl = "confirmed", now
        else:
            status = "issued"
            arm_s = self.ARM_DEADLINE_SIM_S if self._sim() else self.ARM_DEADLINE_S
            dl = now + {"arm": arm_s, "takeoff": arm_s + self.TAKEOFF_EXTRA_S, "disarm": self.DISARM_DEADLINE_S, "land": self.LAND_DEADLINE_S}[name]
        if name == "safe":
            for c in list(self._cmds.values()):
                if c.status == "issued":
                    self._cmds[c.id] = replace(c, status="denied", message="Cancelled by SAFE")
        self._cmds[cid] = CommandState(cid, name, status, res.message, now, dl)
        for old in sorted(self._cmds)[:-50]:
            del self._cmds[old]
        if status == "denied":
            self.core.notify(f"{name.upper().replace('_', ' ')} denied: {res.message}", "caution", "director")
        elif name == "safe":
            self.core.notify(res.message, "info", "director")
        return Result(res.ok, res.message, status, cid)

    def command_status(self, command_id: int) -> Optional[CommandState]:
        return self._cmds.get(command_id)

    def _finish(self, c: CommandState, status: str, msg: str) -> None:
        self._cmds[c.id] = replace(c, status=status, message=msg)
        self.core.notify(f"{c.name.upper().replace('_', ' ')} {'confirmed' if status == 'confirmed' else 'timed out'}: {msg}", "info" if status == "confirmed" else "caution", "director")

    def _update_commands(self, now: float) -> None:
        au = self._au
        for c in [c for c in self._cmds.values() if c.status == "issued"]:
            done = {"arm": au.fc_armed, "takeoff": au.in_flight, "disarm": not au.fc_armed and not au.busy,
                    "land": not au.busy and not au.fc_armed}[c.name]
            if done:
                self._finish(c, "confirmed", {"arm": "Armed", "takeoff": "Lifted off", "disarm": "Disarmed", "land": "Landed"}[c.name])
            elif now > c.deadline_t or (not au.busy and c.name in ("arm", "takeoff") and now - c.started_t > 1.0):
                why = au.message or "no response from the flight controller"
                self._finish(c, "timed_out", f"{why} (not retried: press {c.name.upper()} again when ready)")

    # --- mission phase --------------------------------------------------------------------------
    def _mission_phase(self, now: float) -> str:
        core, au = self.core, self._au
        pub = "disconnected" if not core.connected else ("fault" if core.safety.disarm_latched else au.public_phase())
        if pub in ("disconnected", "fault"):
            mp = "safe"
        elif pub in ("arming", "armed"):
            mp = "armed"
        elif pub == "takeoff":
            mp = "takeoff"
        elif pub == "climbing":
            z = au.height_m()
            mp = "climb" if z is None or z < au.settings.hover_height - 0.05 else "hold"
        elif pub == "holding":
            mp = "hold"
        elif pub == "landing":
            mp = "land"
        else:                                           # ready | landed
            cs = self._pf if self._pf is not None else self.preflight()
            mp = "ready" if (not self._safed and all(c.ok or not c.blocking for c in cs)) else "safe"
        if mp != self._mp[0]:
            self._mp = (mp, now)
        return mp

    @_command("arm")
    def arm(self) -> Result:
        au = self._au
        if au.phase in ("arming", "armed"):
            return Result(False, "already armed" if au.phase == "armed" else "already arming")
        why = self._refusal()
        if why:
            return Result(False, "Cannot arm: " + why)
        au.gyro_cal_before_arm = True
        self._safed = False
        self._grant()
        if not au.arm():
            self._revoke()
            return Result(False, au.message or "Cannot arm")
        return Result(True, "Arming: gyro calibration (keep still), then motors idle")

    @_command("disarm")
    def disarm(self) -> Result:
        return self._disarm()

    def _disarm(self) -> Result:
        au = self._au
        if not self.core.connected:
            return Result(False, "Not connected")
        if au.in_flight:
            return Result(False, "In flight: use LAND, or SAFE to cut the motors")
        if not au.busy and not au.fc_armed:
            return Result(False, "Not armed")
        au.disarm()
        self._revoke()
        return Result(True, "Disarmed")

    @_command("takeoff")
    def takeoff(self, height_m: Optional[float] = None) -> Result:
        au = self._au
        if au.in_flight:
            return Result(False, "Already flying")
        clamp_note = ""
        if height_m is not None:
            ceil = float(au.settings.ceiling_m)
            if float(height_m) > ceil:
                height_m, clamp_note = ceil, f" (target limited to the {ceil:.2f} m ceiling)"
            bad = au.set_height(height_m) if au.phase in ("idle", "armed") else ["busy"]
            if bad and au.phase == "armed":                      # set_height only applies in idle/flying: update the armed settings directly
                bad = au.set_settings(replace(au.settings, hover_height=round(float(height_m), 3)), persist=False)
            if bad:
                return Result(False, "Height refused: " + "; ".join(bad))
        why = self._refusal(skip=("disarmed", "autonomy_idle") if au.phase == "armed" else ())
        if why:
            return Result(False, "Cannot take off: " + why)
        au.gyro_cal_before_arm = True
        was_armed = au.phase == "armed"
        self._safed = False
        self._grant()
        if not au.takeoff():
            if not was_armed:                                        # a refusal must not disarm a drone that is already armed and waiting
                self._revoke()
            return Result(False, au.message or "Cannot take off")
        return Result(True, f"Taking off to {au.settings.hover_height:.2f} m" + clamp_note)

    @_command("land")
    def land(self) -> Result:
        au = self._au
        if au.phase in ("arming", "armed"):
            return self._disarm()
        if not au.in_flight:
            return Result(False, "Not flying")
        au.land("operator")
        return Result(True, "Landing")

    @_command("safe")
    def safe(self) -> Result:
        """Always allowed: disarm, throttle minimum, source idle, arming permission dropped."""
        self.core.safe_now()
        self._au.abort()
        self._granted = False
        self._safed = True
        return Result(True, "SAFE: disarmed, throttle minimum, controls idle")

    @_command("set_height")
    def set_height(self, h: float) -> Result:
        au = self._au
        try:
            h = float(h)
        except (TypeError, ValueError):
            return Result(False, "Height must be a number in metres")
        if h > au.settings.ceiling_m:
            return Result(False, f"Refused: {h:.2f} m is above the ceiling {au.settings.ceiling_m:.2f} m")
        bad = au.set_height(h) if au.phase in ("idle", "flying") else au.set_settings(replace(au.settings, hover_height=round(h, 3)), persist=False)
        return Result(False, "Refused: " + "; ".join(bad)) if bad else Result(True, f"Hold height {h:.2f} m")

    def explore(self, on: bool) -> Result:
        """EXPLORE on/off: the PC explorer steers (nimbus_core/steering.py) while the FC holds the height. Queued during the take-off (starts when the FC holds the height), refused on the ground; LAND / SAFE end it."""
        if not self.core.connected:
            return Result(False, "Not connected")
        ok, msg = self._au.set_explore(bool(on))
        return Result(ok, msg)

    MODE_NAMES = {"stabilize": "STABILIZE: level only, throttle fixed", "alt_hold": "ALT HOLD: holds height, drifts sideways", "pos_hold": "POS HOLD: holds height and position"}

    def set_mode(self, mode: str) -> Result:
        """Choose the flight mode: stabilize | alt_hold | pos_hold. Allowed any time while connected (also in flight); not saved."""
        if not self.core.connected:
            return Result(False, "Not connected")
        if mode not in self.MODE_NAMES:
            return Result(False, f"Unknown flight mode '{mode}': use stabilize, alt_hold or pos_hold")
        if not self._au.set_flight_mode(mode):
            return Result(False, "STABILIZE is not available when the flight controller holds the height: use ALT HOLD or POS HOLD (or LAND)")
        self.core.event(f"FLIGHT MODE -> {mode}")
        return Result(True, "Flight mode " + self.MODE_NAMES[mode])

    def apply_settings(self, new: S.FlightSettings, persist: bool = True) -> Result:
        """Validate, apply and save the settings form in one call."""
        bad = self._au.set_settings(new, persist=persist)
        return Result(False, "Refused: " + "; ".join(bad)) if bad else Result(True, "Settings applied" + (" and saved" if persist else ""))

    # --- state ----------------------------------------------------------------------------------
    def state(self) -> FlightState:
        core, au, t = self.core, self._au, self._tel
        bv, cells, warn = t["batt"], core.safety.cfg.cells, []
        pct = None if bv is None else int(round(max(0.0, min(1.0, (bv / cells - 3.3) / (4.2 - 3.3))) * 100))
        if not core.connected:
            phase = "disconnected"
        elif core.safety.disarm_latched:
            phase = "fault"
        else:
            phase = au.public_phase()
        if bv is not None and bv / cells < core.safety.cfg.batt_warn_v_cell:
            warn.append(f"Battery low: {bv:.1f} V")
        if core.connected and bv is None:
            warn.append("No battery reading")
        if core.safety.disarm_latched:
            warn.append(f"Safety latch: {core.safety.disarm_latched}")
        if phase in ("arming", "armed", "takeoff", "climbing", "holding", "landing") and not self._sim():
            usb = self.preflight_input()["fc_usb"]
            if usb: warn.append("FC USB cable is attached: unplug it while armed")
        now = self.clock()
        mp = self._mission_phase(now)
        return FlightState(phase=phase, armed=au.fc_armed, height_m=au.height_m() if core.connected else None, target_m=au.settings.hover_height,
                           link_lq=t["lq"], battery_v=bv, battery_pct=pct, fc_mode=au.fc_mode,
                           message=au.message, warnings=warn, command=self._cmds.get(self._cmd_seq), mission_phase=mp,
                           phase_elapsed_s=max(0.0, now - self._mp[1]), link_state=core.link.state(), flight_mode=au.flight_mode)
