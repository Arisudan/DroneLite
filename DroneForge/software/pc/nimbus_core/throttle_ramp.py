"""Throttle ramp: finds the throttle at which the drone just lifts (about its hover throttle), the number the hover controller needs.

PROPS ON, DRONE TETHERED (short slack) in a clear area, a person on SAFE / DISARM and a hand near the battery connector.
Arms at zero throttle, then raises throttle slowly (default 1.5 % of stick per second) from a start value, and STOPS (disarms) at the first of:
  lift      range sensor rose above lift_height_m (default 0.12 m)  -> result = throttle at lift-off
  limit     throttle reached the hard cap (default 0.50)           -> not lifted: the cap or tether holds it; nothing is learned
  tilt      |pitch| or |roll| above 25 deg, or the safety layer disarmed
  link/data no range/attitude data for 0.5 s, or the radio link below 20 % LQ
  timeout   max_seconds
Result: lift throttle (stick 0..1). Hover throttle is slightly below it (lift needs thrust > weight); the report gives both."""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

from nimbus_pc.sender import Controls

from . import topics as T


@dataclass
class RampConfig:
    start_thr: float = 0.10
    rate_per_s: float = 0.015
    cap_thr: float = 0.50
    lift_height_m: float = 0.12
    tilt_stop_deg: float = 25.0
    data_stale_s: float = 0.8           # range+flow (0xD3) arrives at ~10 Hz with the FC estimate on
    att_stale_s: float = 2.0            # attitude arrives at ~3 Hz with the FC estimate on (was 7 Hz)
    max_seconds: float = 60.0
    arm_settle_s: float = 1.5
    soft_start_per_s: float = 0.30      # throttle rises from 0 to the start value at most this fast (no step to a high start value)
    hold_at_cap_s: float = 3.0          # after reaching the cap, hold it this long (so you can SEE whether it lifts at exactly that throttle)
    gyro_cal_s: float = 4.0             # before arming: the gyro-calibration stick gesture (throttle min, yaw left, pitch back, roll centre, DISARMED) then stillness; 0 = off (sim/bench always skip it)
    arm_timeout_s: float = 8.0          # the FC needs ~1-2 s of gyro calibration (CALIB) before it arms; a moving/vibrating drone makes it longer


class ThrottleRamp:
    def __init__(self, core, cfg: Optional[RampConfig] = None) -> None:
        self.core, self.cfg = core, cfg or RampConfig()
        self.phase = "idle"                 # idle | arming | ramping | done
        self.result: Optional[dict] = None
        self.log: list = []
        self._t0 = self._t_ramp = 0.0
        self._range = self._pitch = self._roll = None
        self._t_range = self._t_att = None
        self._lq = None
        self._base_range = None
        self._t_cap = None
        self.thr = 0.0
        core.bus.subscribe(T.TELEMETRY, self._on_tel)

    def _on_tel(self, msg) -> None:
        import math
        name, d = msg; now = time.monotonic()
        if name == "optrange":
            self._range, self._t_range = d["distance_m"], now
        elif name == "attitude":
            self._pitch, self._roll, self._t_att = math.degrees(d["pitch_rad"]), math.degrees(d["roll_rad"]), now
        elif name == "link":
            self._lq = d["up_lq"]

    def start(self) -> bool:
        c = self.core
        if self.phase in ("arming", "ramping") or not c.connected or not c.allow_arm or c.source != "external":
            return False
        self.phase, self.result, self.log, self.thr = "arming", None, [], 0.0
        self._t0 = time.monotonic(); self._base_range = None; self._t_cap = None
        c.event("RAMP started: arming at zero throttle (props ON, tethered)")
        return True

    def stop(self, reason: str) -> None:
        if self.phase in ("done", "idle"):
            return
        self.phase = "done"
        lift = self.thr if reason == "lift" else None
        vb = self.core.safety.vbat_f
        self.result = {"reason": reason, "batt_v": (round(vb, 2) if vb else None), "lift_throttle": lift, "hover_throttle_est": (lift * 0.95 if lift else None),
                       "last_throttle": self.thr, "seconds": time.monotonic() - self._t0, "samples": len(self.log)}
        self.core.set_external_controls(Controls())
        self.core.event(f"RAMP stopped ({reason}); lift throttle {lift if lift is None else round(lift, 3)}")

    def tick(self, now: float) -> Controls:
        c, cfg = self.core, self.cfg
        if self.phase in ("idle", "done"):
            return Controls()
        if now - self._t0 > cfg.max_seconds:
            self.stop("timeout"); return Controls()
        if c.safety.disarm_latched:
            self.stop("safety_disarm:" + c.safety.disarm_latched); return Controls()
        if self._lq is not None and self._lq < 20:
            self.stop("link"); return Controls()
        if self.phase == "arming":
            gc = 0.0 if c.state_source in ("sim", "bench") else cfg.gyro_cal_s
            el = now - self._t0
            if el < gc:                                     # re-zero the gyro offset right before arming (disarmed, throttle at minimum: the motors cannot spin)
                if el < 1.6 and not c.autonomy.fc_armed:
                    return Controls(pitch=-1.0, yaw=-1.0)
                return Controls()
            el -= gc
            if c.autonomy.fc_armed and el >= cfg.arm_settle_s:
                self.phase, self._t_ramp = "ramping", now
            elif el > cfg.arm_timeout_s + cfg.arm_settle_s:
                self.stop("did_not_arm"); return Controls()
            return Controls(arm=True)
        # ramping
        age_r = None if self._t_range is None else now - self._t_range
        age_a = None if self._t_att is None else now - self._t_att
        if age_r is None or age_r > cfg.data_stale_s or age_a is None or age_a > cfg.att_stale_s:
            self.stop("no_data:" + ("range %.2fs" % age_r if age_r is None or age_r > cfg.data_stale_s else "") + (" attitude %.2fs" % age_a if age_a is None or age_a > cfg.att_stale_s else "")); return Controls()
        if self._base_range is None:
            self._base_range = self._range
        if max(abs(self._pitch), abs(self._roll)) > cfg.tilt_stop_deg:
            self.stop("tilt"); return Controls()
        self.thr = min(cfg.start_thr + cfg.rate_per_s * (now - self._t_ramp), cfg.cap_thr, cfg.soft_start_per_s * (now - self._t_ramp))
        self.log.append((round(now - self._t0, 2), round(self.thr, 3), round(self._range, 3), round(self._pitch, 1), round(self._roll, 1)))
        if self._range - self._base_range > cfg.lift_height_m:
            self.stop("lift"); return Controls()
        if self.thr >= cfg.cap_thr:
            self._t_cap = self._t_cap if self._t_cap is not None else now
            if now - self._t_cap >= cfg.hold_at_cap_s:
                self.stop("limit"); return Controls()
        return Controls(throttle=self.thr, arm=True)
