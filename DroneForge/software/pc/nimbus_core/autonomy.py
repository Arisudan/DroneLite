"""Hover autonomy inside the core (Nimbus: AutonomyRequest Takeoff / Land, AutonomyStatus). Wraps nimbus_pc.hover.HoverController.

Flow: idle -> (takeoff) arming [arm HIGH at ZERO throttle until the FC reports armed; the real Betaflight refuses otherwise]
      -> flying [HoverController: takeoff, sign check, hover, land] -> idle.
The core calls controls() each tick while the control source is "external" and the phase is arming/flying. Telemetry (range/flow/attitude/battery)
comes from the topic bus, so it works the same on the live link and on the Sim state source. Safety land requests are honoured here."""
from __future__ import annotations

import math
import threading
import time
from dataclasses import replace
from typing import Optional

from nimbus_pc.hover.controller import HoverConfig, HoverController, HoverState
from nimbus_pc.hover.fc_hold_controller import FcHoldController
from nimbus_pc.hover.sim import Sample
from nimbus_pc.mapper import MODE_ALT_TICKS
from nimbus_pc.sender import Controls

from . import settings as S
from .steering import SteerLink
from . import topics as T


def to_sample(name: str, d: dict, now: float) -> Optional[Sample]:
    if name == "optrange":
        return Sample(now, "optrange", {"distance_m": d["distance_m"], "flow_vx": d["velocity_forward"], "flow_vy": d["velocity_right"], "quality": d["flow_quality"]})
    if name == "attitude":
        return Sample(now, "attitude", {"pitch_deg": math.degrees(d["pitch_rad"]), "roll_deg": math.degrees(d["roll_rad"]), "yaw_deg": math.degrees(d["yaw_rad"])})
    if name == "kinematic" and (d.get("valid") or "hold_active" in d):
        att = d.get("yaw_rad", 0.0)
        return Sample(now, "fcxy", {"vx_mps": d["vx_mps"], "vy_mps": d["vy_mps"], "yaw_deg": 0.0 if d.get("vel_body") else math.degrees(att), "valid": bool(d.get("valid")), "vel_body": bool(d.get("vel_body", False)), "xy_disabled": bool(d.get("xy_disabled", False)),
                                    "hold_active": d.get("hold_active", False), "hold_fault": d.get("hold_fault", False), "hold_mode": d.get("hold_mode", 0)})
    if name == "battery":
        return Sample(now, "battery", {"voltage_v": d["voltage_v"]})
    return None


class HoverAutonomy:
    ARM_TIMEOUT_S = 8.0
    GYRO_CAL_S = 4.0                         # gyro-calibration gesture + stillness before the arm command
    COUNTDOWN_S = 3.0                        # 3-2-1 before the arm command on the real drone (Nimbus Start sequence: gyro cal -> countdown -> estimator reset -> arm)

    def __init__(self, core, cfg: Optional[HoverConfig] = None) -> None:
        self.core = core
        self.cfg = cfg or HoverConfig()
        self.settings = S.load()
        S.apply_to_config(self.settings, self.cfg)
        core.safety.cfg.max_height_m = self.settings.ceiling_m
        self.ctl = self._new_ctl()
        self.flight_mode = "pos_hold"            # not saved: back to pos_hold when a flight ends
        self.phase = "idle"                      # idle | arming | armed | flying   ("armed" = FC armed at zero throttle, motors at idle, waiting for take-off)
        self._arm_only = False; self._t_armlost = None
        self.fc_mode = ""
        self._est_reset = False
        self.gyro_cal_before_arm = True           # recalibrate the FC gyro right before every arming (stick gesture, disarmed)
        self.att_deg = None                      # (pitch, roll, time) the FC reports: at rest on the ground this is the "rest tilt"
        self.message = ""
        self._lock = threading.RLock()
        self._buf: list = []
        self._t_last: Optional[float] = None
        self._t_arm: Optional[float] = None
        self.steer = SteerLink()                 # EXPLORE: stick wishes from the PC autonomy (nimbus_core/steering.py); the listener starts on the first EXPLORE
        self.explore = False
        core.bus.subscribe(T.TELEMETRY, self._on_telemetry)

    # --- inputs -------------------------------------------------------------------------------
    def _on_telemetry(self, msg) -> None:
        name, d = msg
        if name == "attitude":
            import math as _m
            self.att_deg = (_m.degrees(d["pitch_rad"]), _m.degrees(d["roll_rad"]), time.monotonic())
        if name == "flight_mode":
            self.fc_mode = d.get("mode", "")
            return
        now = time.monotonic()
        s = to_sample(name, d, now)
        if s is None:
            return
        with self._lock:
            if self.phase == "flying":
                self._buf.append(s)
            else:
                self.ctl.observe(now, [s])            # keeps the telemetry-rate/battery judgement fresh for the preflight

    @property
    def fc_armed(self) -> bool:
        return bool(self.fc_mode) and not self.fc_mode.endswith("*")

    # --- commands -----------------------------------------------------------------------------
    def preflight(self, for_ramp: bool = False) -> tuple[bool, list]:
        """for_ramp=True skips only the 'hover throttle must be measured' rule (the ramp is what measures it)."""
        with self._lock:
            ok, why = self.ctl.preflight(time.monotonic())
        if not self.core.connected:
            ok, why = False, ["not connected"] + why
        if not self.core.allow_arm:
            ok, why = False, ["arming is not permitted (the flight director grants it)"] + why
        if self.core.safety.disarm_latched:
            ok, why = False, [f"safety disarm latched ({self.core.safety.disarm_latched})"] + why
        if not for_ramp and not self.settings.measured and not (self.core.state_source in ("sim", "bench")):
            ok, why = False, ["hover throttle is UNMEASURED: set the hover throttle and reference voltage from a real lift-off, then set Settings > Controller > Hover throttle status to Measured"] + why
        if self.core.source != "external":
            ok, why = False, [f"controls source is '{self.core.source}', not external"] + why
        return ok, why

    def _new_ctl(self):
        """The height controller for the saved settings: the FC-side hybrid or the PC loop."""
        return FcHoldController(self.cfg) if self.settings.profile == "fc" else HoverController(self.cfg)

    def set_settings(self, s: S.FlightSettings, persist: bool = True) -> list:
        """Validate, apply to the hover config and (optionally) save. Refused while a flight is running."""
        if self.phase not in ("idle", "armed"):
            return ["not while flying"]
        bad = s.validate()
        if bad:
            return bad
        self.settings = s
        S.apply_to_config(s, self.cfg)
        self.core.safety.cfg.max_height_m = s.ceiling_m
        old = self.ctl
        self.ctl = self._new_ctl()
        for a in ("opt_times", "_raw_hist", "last_optrange", "batt_v", "ground", "pitch_deg", "roll_deg", "imu_az", "t_imu"):   # keep the live telemetry judgement: applying settings must not blind the preflight
            setattr(self.ctl, a, getattr(old, a))
        if persist:
            S.save(s)
        self.core.event(f"AUTONOMY settings: hover {s.hover_height:.2f} m, hover throttle {s.thr_hover0:.3f} ({'measured' if s.measured else 'UNMEASURED default'}), cap {s.thr_max:.2f}")
        return []

    def set_height(self, h: float) -> list:
        """Change the hold height NOW (not saved as the default). Idle/arming: used by the next takeoff. Flying: the drone glides to the new height."""
        from dataclasses import replace
        h = round(float(h), 3)
        with self._lock:
            new = replace(self.settings, hover_height=h)
            bad = new.validate()
            if bad:
                return bad
            if self.phase == "idle":
                return self.set_settings(new, persist=False)
            if self.phase == "flying" and not self.ctl.command_height(time.monotonic(), h):
                return ["not while landing / after a sensor loss"]
            self.settings = new
            self.cfg.hover_height = h
            self.cfg.ceiling = max(self.cfg.ceiling, h + 0.3)
            self.core.event(f"AUTONOMY hold height -> {h:.2f} m")
            return []

    def arm(self) -> bool:
        """ARM only: gyro calibration (disarmed), then the FC is armed at ZERO throttle (motors at idle) and stays armed until disarm() or take-off."""
        if self.phase != "idle":
            self.message = "already running"; return False
        why = []
        if not self.core.connected: why.append("not connected")
        if not self.core.allow_arm: why.append("arming is not permitted (the flight director grants it)")
        if self.core.safety.disarm_latched: why.append(f"safety disarm latched ({self.core.safety.disarm_latched})")
        if self.core.source != "external": why.append(f"controls source is '{self.core.source}', not external")
        if why:
            self.message = "arming refused: " + "; ".join(why); self.core.event("AUTONOMY " + self.message); return False
        with self._lock:
            self.phase, self._t_arm, self._t_last, self._buf, self._arm_only, self._t_armlost, self._est_reset = "arming", time.monotonic(), None, [], True, None, False
        self.message = "arming at zero throttle (motors will idle)"
        self.core.event("AUTONOMY ARM requested: arming at zero throttle")
        return True

    def _drain_ctl_events(self) -> None:
        """Forward EVERY new hover-controller event (state changes, land / abort reasons, gate events, learned hover, drop-outs, ...) to the core event stream, with the controller state and
        the key numbers at that moment: it reaches the recorded events.txt and the GCS notifications. Land / abort reasons also become the status message (plain words)."""
        ctl = self.ctl
        i = getattr(ctl, "_ev_sent", 0)
        evs = ctl.events
        if i >= len(evs):
            return
        ctl._ev_sent = len(evs)
        try:
            tilt = max(abs(ctl.pitch_deg), abs(ctl.roll_deg))
            rng = "-" if getattr(ctl, "last_range_m", None) is None else f"{ctl.last_range_m:.2f} m"
            ctx = f" [state {ctl.state.value}, z_est {ctl.z:.2f} m, vz {ctl.kz.x[1]:+.2f} m/s, range {rng}, throttle {ctl.cmd.throttle:.2f}, tilt {tilt:.0f} deg]"
        except Exception:
            ctx = ""
        for _t, text in evs[i:]:
            text = str(text)
            self.core.event("HOVER " + text + ctx)
            if "-> land (" in text or "-> sensor_loss" in text or "-> done (" in text and "landed" not in text:
                why = text.split("(", 1)[1][:-1] if "(" in text and text.endswith(")") else text
                self.message = ("Landing: " if "-> land" in text else "Stopped: ") + why

    def _learn_hover_after_flight(self) -> None:
        """After a flight that settled >= 3 s in hover (not aborted): move the persisted hover throttle towards what this flight needed (EMA, bounded; see settings.learn_hover).
        The next take-off then starts its ground ramp at the learned value. measured / batt_ref_v are kept; a failure here never affects the flight."""
        try:
            got = self.ctl.hover_learned_flight()
            if got is None:
                return
            res = S.learn_hover(self.settings, got[0])
            if res is None:
                return
            new, text = res
            S.save(new)
            self.settings = new
            S.apply_to_config(new, self.cfg)
            self.core.event("AUTONOMY " + text)
        except (OSError, ValueError) as e:
            self.core.event(f"AUTONOMY hover throttle not saved: {e}")

    def _reset_estimator(self) -> None:
        """New hover controller (fresh filters) that keeps the live telemetry history, like set_settings does."""
        old = self.ctl
        self.ctl = self._new_ctl()
        for a in ("opt_times", "_raw_hist", "last_optrange", "batt_v", "ground", "pitch_deg", "roll_deg", "imu_az", "t_imu"):
            setattr(self.ctl, a, getattr(old, a))

    def set_flight_mode(self, mode: str) -> bool:
        if mode not in HoverController.MODES or not self.ctl.set_flight_mode(mode):
            return False
        self.flight_mode = mode
        return True

    def disarm(self) -> None:
        with self._lock:
            if self.phase in ("arming", "armed"):
                self.phase = "idle"; self._arm_only = False; self.message = "disarmed"
        self.core.event("AUTONOMY disarm requested")

    def takeoff(self) -> bool:
        ok, why = self.preflight()
        if self.phase not in ("idle", "armed"):
            self.message = "already running"; return False
        if not ok:
            self.message = "takeoff refused: " + "; ".join(why)
            self.core.event("AUTONOMY " + self.message)
            return False
        with self._lock:
            if self.phase == "armed":                                  # already armed: straight into the take-off
                if self.ctl.start(time.monotonic()):
                    self.phase, self._t_last, self._buf, self._arm_only = "flying", None, [], False
                    self.message = "flying"; self.core.event("AUTONOMY take-off from the armed state: hover controller started")
                    return True
                self.message = "takeoff refused: " + "; ".join(self.ctl.refused); self.core.event("AUTONOMY " + self.message)
                self._drain_ctl_events()
                return False
            self.phase, self._t_arm, self._t_last, self._buf, self._arm_only, self._est_reset = "arming", time.monotonic(), None, [], False, False
        self.message = "arming at zero throttle"
        self.core.event("AUTONOMY takeoff requested: arming at zero throttle")
        return True

    def set_explore(self, on: bool) -> tuple:
        """EXPLORE on/off. On only while flying and the height hold is holding; the steering then comes from the explorer process (centred sticks while it sends nothing)."""
        with self._lock:
            if not on:
                was = self.explore; self.explore = False; self.steer.active = False
                if was: self.core.event("AUTONOMY explore stopped (operator)")
                return True, "Explore stopped: the FC holds the position"
            if self.phase != "flying" or self.ctl.state != HoverState.HOVER:
                return False, "Explore needs a steady hover first (take off and wait for HOLDING)"
            if not self.steer.start():
                return False, self.steer.error
            self.steer.done = False; self.explore = True; self.steer.active = True
        self.core.event("AUTONOMY explore started: steering from the explorer (udp 127.0.0.1:%d)" % self.steer.port)
        return True, "Explore on: the explorer steers, the FC holds the height. LAND or SAFE end it."

    def _explore_off(self, why: str) -> None:
        if self.explore:
            self.explore = False; self.steer.active = False; self.core.event(f"AUTONOMY explore stopped ({why})")

    def land(self, why: str = "operator") -> None:
        with self._lock:
            self._explore_off("landing")
            if self.phase == "arming":
                self.phase = "idle"; self.message = "cancelled before arming"
            elif self.phase == "flying":
                self.ctl.land(time.monotonic(), why)
                self._drain_ctl_events()
                self.message = f"landing ({why})"
        self.core.event(f"AUTONOMY land requested ({why})")

    def abort(self) -> None:
        with self._lock:
            self._explore_off("abort")
            self.phase = "idle"; self._buf = []
            self.ctl = self._new_ctl(); self.flight_mode = "pos_hold"
        self.message = "aborted"

    # --- control tick (called by the core while source == external and phase in arming/flying) --
    def controls(self, now: float) -> Controls:
        with self._lock:
            if self.phase == "arming":
                real = self.core.state_source not in ("sim", "bench")
                gc = 0.0 if (not self.gyro_cal_before_arm or not real) else self.GYRO_CAL_S
                cd = self.COUNTDOWN_S if (real and not self.fc_armed) else 0.0           # no countdown if the FC is somehow already armed
                el = now - (self._t_arm or now)
                if el < gc:                                 # 1) re-zero the gyro offset right before arming (disarmed, throttle at minimum: the motors cannot spin)
                    self.message = f"Gyro calibration: keep the drone still ({gc - el:.0f} s)"
                    if el < 1.6 and not self.fc_armed:
                        return Controls(pitch=-1.0, yaw=-1.0)
                    return Controls()
                if el < gc + cd:                            # 2) countdown, still disarmed
                    self.message = f"Starting in {max(1, math.ceil(gc + cd - el))}..."
                    return Controls()
                if cd and not self._est_reset:              # 3) fresh estimator state right before the arm command (keeps the live telemetry judgement)
                    self._reset_estimator(); self._est_reset = True
                    self.core.event("AUTONOMY start sequence: gyro calibrated, countdown done, estimator reset: arming")
                gc += cd
                if self.fc_armed and self._arm_only:
                    self.phase, self.message, self._t_armlost = "armed", "ARMED: motors at idle, ready to take off", None
                    self.core.event("AUTONOMY ARMED: the FC is armed at zero throttle (motors idle)")
                    return Controls(arm=True)
                if self.fc_armed:
                    if self.ctl.start(now):
                        self.phase, self.message = "flying", "flying"
                        self.core.event("AUTONOMY armed: hover controller started")
                    else:
                        self.phase, self.message = "idle", "takeoff refused: " + "; ".join(self.ctl.refused)
                        self._drain_ctl_events()
                        self.core.event("AUTONOMY " + self.message)
                        return Controls()
                elif el - gc > self.ARM_TIMEOUT_S:
                    self.phase, self.message = "idle", "FC did not arm within 8 s (gyro calibration needs the drone STILL; check the arming blockers / Boxer SA / SC)"
                    self.core.event("AUTONOMY " + self.message)
                    return Controls()
                return Controls(arm=True)             # zero throttle
            if self.phase == "armed":
                if self.fc_armed:
                    self._t_armlost = None
                else:                                                 # the FC dropped out of armed (safety, failsafe, link): go back to idle
                    self._t_armlost = self._t_armlost if self._t_armlost is not None else now
                    if now - self._t_armlost > 1.5:
                        self.phase, self.message = "idle", "the FC is no longer armed"; self.core.event("AUTONOMY " + self.message)
                        return Controls()
                return Controls(arm=True)
            if self.phase != "flying":
                return Controls()
            dt = 0.02 if self._t_last is None else min(max(now - self._t_last, 0.005), 0.1)
            self._t_last = now
            if self.core.safety.land_request and self.ctl.state in (HoverState.TAKEOFF, HoverState.SIGN_CHECK, HoverState.HOVER):
                self.ctl.land(now, "safety: " + self.core.safety.land_request)
                self._drain_ctl_events()
                self.message = f"landing (safety: {self.core.safety.land_request})"
            samples, self._buf = self._buf, []
            self.ctl.set_flight_mode(self.flight_mode)
            cmd = self.ctl.update(now, samples, dt)
            self._drain_ctl_events()
            if self.explore:
                if self.ctl.state != HoverState.HOVER:
                    self._explore_off(f"height hold left HOVER ({self.ctl.state.value})")
                elif self.steer.done:
                    why = self.steer.done_reason or "finished"
                    self._explore_off(f"the explorer stopped: {why}"); self.message = f"explore stopped ({why}): holding. EXPLORE = search again, or LAND"
                else:
                    st = self.steer.latest()
                    if st is not None:                                  # stale or absent: centred sticks + POS HOLD = the FC holds the position
                        cmd = replace(cmd, roll=st[0], pitch=st[1], yaw=st[2], aux7=MODE_ALT_TICKS)      # ALT HOLD on AUX7: the FC pauses its sideways loop, the sticks are pure tilt (fc firmware v4.2+)
            if self.ctl.state == HoverState.DONE:
                self._learn_hover_after_flight()
                self.core.event(f"AUTONOMY finished: {self.ctl.reason or 'landed'}")
                self.message = f"finished: {self.ctl.reason or 'landed'}"
                self.phase = "idle"
                self.ctl = self._new_ctl(); self.flight_mode = "pos_hold"
                return Controls()
            return cmd

    # --- what the flight director needs (no GUI knowledge) -----------------------------------
    @property
    def busy(self) -> bool:
        return self.phase != "idle"

    @property
    def in_flight(self) -> bool:
        return self.phase in ("flying",)

    def height_m(self) -> Optional[float]:
        """Latest height estimate in metres (None before any range data)."""
        with self._lock:
            return self.ctl.z if self.ctl.z is not None else None

    def public_phase(self) -> str:
        """arming|armed|takeoff|climbing|holding|landing|landed|ready|fault (connection state is the director's business)."""
        with self._lock:
            if self.phase == "arming": return "arming"
            if self.phase == "armed": return "armed"
            if self.phase == "flying":
                st = self.ctl.state
                if st == HoverState.LAND: return "landing"
                if st == HoverState.HOVER: return "holding"
                if st == HoverState.SENSOR_LOSS: return "fault"
                return "takeoff" if (self.ctl.z or 0.0) < 0.10 else "climbing"
            return "landed" if self.message.startswith("finished") else "ready"

    # --- status for the UI --------------------------------------------------------------------
    def status(self) -> dict:
        with self._lock:
            now = time.monotonic()
            rate, gap = self.ctl.telemetry_health(now)
            return {"phase": self.phase, "state": self.ctl.state.value, "z_m": self.ctl.z, "range_hz": rate, "gap_s": gap,
                    "batt_v": self.ctl.batt_v, "reason": self.ctl.reason, "message": self.message,
                    "events": [str(e[1]) for e in self.ctl.events[-4:]], "fc_armed": self.fc_armed,
                    "xy_source": self.ctl.xy_active, "explore": self.explore, "steer_hz": self.steer.rate_hz() if self.steer.running else 0.0,
                    "rest_tilt": (None if (self.att_deg is None or now - self.att_deg[2] > 3.0) else (self.att_deg[0], self.att_deg[1]))}
