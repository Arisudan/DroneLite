"""Safety layer (Nimbus-style): tilt disarm, low-battery warning/landing, ceiling guard, lost-sensor landing, landing throttle ramp.

Pure logic, no I/O: feed it telemetry, call check() each control tick. It returns events and sets two latches:
  disarm_latched  -> the core forces arm LOW + throttle minimum until the operator's arm request goes low again (reset())
  land_request    -> the active controller (hover/autonomy) must land; manual pilots only get the warning
Tilt reasons mirror Nimbus's DroneTiltExceededReason: SustainedAttitude, ExtremeAttitude, RawImuInverted.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional


@dataclass
class SafetyConfig:
    cells: int = 4
    tilt_sustained_deg: float = 45.0       # tilt beyond this for tilt_sustained_s -> disarm
    tilt_sustained_s: float = 0.5
    tilt_extreme_deg: float = 70.0         # instant disarm
    inverted_deg: float = 110.0            # |roll| or |pitch| beyond this = flipped over (RawImuInverted proxy: attitude)
    attitude_fresh_s: float = 1.0          # tilt checks only trust attitude younger than this
    batt_autoland: bool = False            # False (default): low battery only WARNS (event/banner), no landing request; True: land request below batt_land_v_cell
    batt_warn_v_cell: float = 3.55
    batt_land_v_cell: float = 3.40
    batt_hold_s: float = 3.0               # filtered voltage must stay below the limit this long (ignores throttle punch sag)
    batt_filter_tau_s: float = 2.0
    max_height_m: float = 3.0              # ceiling/perimeter guard (indoor)
    sensor_loss_s: float = 1.0             # no range frame for this long while airborne -> land
    link_lq_min: float = 20.0
    link_loss_s: float = 1.5               # radio link below link_lq_min (or no link frame) this long while armed -> land
    airborne_height_m: float = 0.10


@dataclass
class SafetyEvent:
    t: float
    code: str                              # e.g. tilt_extreme, tilt_sustained, inverted, battery_warn, battery_land, ceiling, sensor_lost, link_lost
    level: str                             # "warn" | "land" | "disarm"
    text: str


@dataclass
class SafetyMonitor:
    cfg: SafetyConfig = field(default_factory=SafetyConfig)

    def __post_init__(self) -> None:
        self.pitch = self.roll = 0.0
        self.t_att: Optional[float] = None
        self.vbat_f: Optional[float] = None
        self.t_batt: Optional[float] = None
        self.range_m: Optional[float] = None
        self.t_range: Optional[float] = None
        self.lq: Optional[float] = None
        self.t_link: Optional[float] = None
        self.disarm_latched: Optional[str] = None
        self.land_request: Optional[str] = None
        self.events: list[SafetyEvent] = []
        self._tilt_since: Optional[float] = None
        self._batt_low_since: Optional[float] = None
        self._link_low_since: Optional[float] = None
        self._warned: set = set()

    # --- inputs ------------------------------------------------------------------------------
    def feed_attitude(self, now: float, pitch_deg: float, roll_deg: float) -> None:
        self.pitch, self.roll, self.t_att = pitch_deg, roll_deg, now

    def feed_battery(self, now: float, volts: float) -> None:
        if self.vbat_f is None or self.t_batt is None:
            self.vbat_f = volts
        else:
            a = min((now - self.t_batt) / self.cfg.batt_filter_tau_s, 1.0)
            self.vbat_f += a * (volts - self.vbat_f)
        self.t_batt = now

    def feed_range(self, now: float, metres: float) -> None:
        self.range_m, self.t_range = metres, now

    def feed_link(self, now: float, lq: float) -> None:
        self.lq, self.t_link = lq, now

    # --- evaluation --------------------------------------------------------------------------
    def reset(self) -> None:
        """Operator's arm request went low: clear the latches (a new arming starts clean)."""
        self.disarm_latched = None
        self.land_request = None
        self._tilt_since = None
        self._warned.discard("ceiling")

    def _emit(self, out: list, ev: SafetyEvent) -> None:
        out.append(ev); self.events.append(ev)
        del self.events[:-200]

    def check(self, now: float, armed: bool, turtle: bool = False) -> list[SafetyEvent]:
        c, out = self.cfg, []
        # --- tilt (never while the operator deliberately flips with turtle mode) ---
        att_ok = self.t_att is not None and now - self.t_att <= c.attitude_fresh_s
        if armed and att_ok and not turtle and self.disarm_latched is None:
            tilt = max(abs(self.pitch), abs(self.roll))
            if tilt >= c.inverted_deg:
                self.disarm_latched = "inverted"
                self._emit(out, SafetyEvent(now, "inverted", "disarm", f"drone is upside down (pitch {self.pitch:+.0f}, roll {self.roll:+.0f} deg): disarm"))
            elif tilt >= c.tilt_extreme_deg:
                self.disarm_latched = "tilt_extreme"
                self._emit(out, SafetyEvent(now, "tilt_extreme", "disarm", f"tilt {tilt:.0f} deg >= {c.tilt_extreme_deg:.0f}: disarm"))
            elif tilt >= c.tilt_sustained_deg:
                self._tilt_since = self._tilt_since if self._tilt_since is not None else now
                if now - self._tilt_since >= c.tilt_sustained_s:
                    self.disarm_latched = "tilt_sustained"
                    self._emit(out, SafetyEvent(now, "tilt_sustained", "disarm", f"tilt {tilt:.0f} deg for {now - self._tilt_since:.1f} s: disarm"))
            else:
                self._tilt_since = None
        elif not armed:
            self._tilt_since = None
        # --- battery ---
        if self.vbat_f is not None:
            per_cell = self.vbat_f / c.cells
            if per_cell <= c.batt_land_v_cell:
                self._batt_low_since = self._batt_low_since if self._batt_low_since is not None else now
                if armed and now - self._batt_low_since >= c.batt_hold_s and "battery_land" not in self._warned:
                    if c.batt_autoland and self.land_request is None:
                        self.land_request = "battery"
                        self._emit(out, SafetyEvent(now, "battery_land", "land", f"battery {self.vbat_f:.2f} V ({per_cell:.2f} V/cell) below {c.batt_land_v_cell:.2f}: land now"))
                    else:
                        self._warned.add("battery_land")
                        self._emit(out, SafetyEvent(now, "battery_land", "warn", f"battery {self.vbat_f:.2f} V ({per_cell:.2f} V/cell) below {c.batt_land_v_cell:.2f}: LAND SOON (warning only, you decide)"))
            else:
                self._batt_low_since = None
                self._warned.discard("battery_land")
            if per_cell <= c.batt_warn_v_cell and "battery_warn" not in self._warned:
                self._warned.add("battery_warn")
                self._emit(out, SafetyEvent(now, "battery_warn", "warn", f"battery low: {self.vbat_f:.2f} V ({per_cell:.2f} V/cell)"))
            elif per_cell > c.batt_warn_v_cell + 0.10:
                self._warned.discard("battery_warn")
        # --- ceiling / perimeter ---
        if armed and self.range_m is not None and self.t_range is not None and now - self.t_range < 1.0 and self.range_m > c.max_height_m:
            if "ceiling" not in self._warned:
                self._warned.add("ceiling")
                self.land_request = self.land_request or "ceiling"
                self._emit(out, SafetyEvent(now, "ceiling", "land", f"height {self.range_m:.2f} m above the {c.max_height_m:.1f} m limit: land"))
        # --- lost range sensor while flying ---
        if armed and self.t_range is not None and self.range_m is not None and self.range_m > c.airborne_height_m \
                and now - self.t_range > c.sensor_loss_s and "sensor_lost" not in self._warned:
            self._warned.add("sensor_lost"); self.land_request = self.land_request or "sensor_lost"
            self._emit(out, SafetyEvent(now, "sensor_lost", "land", f"no range data for {now - self.t_range:.1f} s while flying: land (blind descent)"))
        elif self.t_range is not None and now - self.t_range < 0.5:
            self._warned.discard("sensor_lost")
        # --- radio link ---
        if armed:
            lost = self.t_link is None or now - self.t_link > c.link_loss_s or (self.lq is not None and self.lq < c.link_lq_min)
            if lost:
                self._link_low_since = self._link_low_since if self._link_low_since is not None else now
                if now - self._link_low_since >= c.link_loss_s and "link_lost" not in self._warned:
                    self._warned.add("link_lost"); self.land_request = self.land_request or "link_lost"
                    self._emit(out, SafetyEvent(now, "link_lost", "land", "radio link lost or very weak while armed: land"))
            else:
                self._link_low_since = None; self._warned.discard("link_lost")
        else:
            self._link_low_since = None; self._warned.discard("link_lost")
        return out

    def status(self) -> dict:
        return {"disarm_latched": self.disarm_latched, "land_request": self.land_request,
                "battery_v_filtered": self.vbat_f, "tilt_deg": max(abs(self.pitch), abs(self.roll)),
                "recent": [(e.code, e.level) for e in self.events[-5:]]}


def landing_throttle(t_since_start: float, thr_start: float, thr_idle: float = 0.0, ramp_s: float = 2.0) -> float:
    """Landing throttle ramp: linear from the throttle when the ramp starts down to idle over ramp_s (Nimbus: landing-throttle-ramp)."""
    if t_since_start <= 0:
        return thr_start
    if t_since_start >= ramp_s:
        return thr_idle
    return thr_idle + (thr_start - thr_idle) * (1.0 - t_since_start / ramp_s)
