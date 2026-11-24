"""Hybrid controller: the FC runs the height loop (firmware flight/height_hold*.c: take-off creep, hold, landing), the PC only requests it and supervises.

Plugs into the same slot as HoverController (nimbus_core.autonomy picks it when FlightSettings.profile == "fc"), so the director, the GCS buttons, the preflight list, the events and the
recorder work unchanged. What it sends: throttle stick at minimum, level sticks, AUX5 = switch + target height (mapper.height_to_aux5, AUX5_LAND to land), AUX6 = hover-reference
throttle. What it reads: the range (height above the ground reference taken at start) and the FC's hold status bits in the 0xD6 state frame (active, fault, mode).
Why: the PC loop has a 0.4-0.55 s throttle-to-range delay (radio, FC, motors) that no PC-side tuning survives (docs/maintenance/REAL_FLIGHT_FIT_2026-11-13.md); the FC loop has none of the radio path.
The FC also holds the position sideways (v4 firmware); the PC sends level sticks. The FC's own landing is 0.12 m/s (about 0.7 m/s at the floor in the sim)."""
from __future__ import annotations

from typing import Optional

from nimbus_pc.mapper import AUX5_LAND, height_to_aux5, mode_to_aux7, throttle01_to_crsf
from nimbus_pc.sender import CRSF_MIN, Controls

from .controller import HoverConfig, HoverController, HoverState

TAKEOFF_TIMEOUT_S = 40.0      # the FC creep alone takes ~10-25 s
STATUS_MAX_AGE_S = 1.0        # the hold status must be this fresh, or the firmware has no hold status frame (old build)
LIFTED_M = 0.15
ARRIVED_M = 0.06              # HOVER (GCS "holding") only within this of the target height, calm for 1 s
AIR_GRACE_S = 8.0             # ... or after this long airborne in the FC hold, whatever the height (the take-off timeout never acts in the air)

class FcHoldController(HoverController):
    def __init__(self, cfg: Optional[HoverConfig] = None):
        super().__init__(cfg)
        self.fc_active = False; self.fc_fault = False; self.fc_mode = 0; self.fc_status_t = -1e9
        self._z_f = 0.0; self._t_req = 0.0; self._fallback = False; self._saw_land_mode = False
        self.hold_ref = None
        self._zh = []; self._t_air = None
        self._xy_note = False
        self.xy_active = "fc"                                 # the sideways hold runs on the FC (flight/xy_hold_core.c): speed, position and level trim in the drone's own axes

    def set_flight_mode(self, mode: str) -> bool:
        """With the FC hold only ALT HOLD (height) and POS HOLD (height + position) exist: the FC gets the mode on AUX7. STABILIZE (no height control) is refused: the PC has no height loop here."""
        if mode not in ("alt_hold", "pos_hold"):
            return False
        self.flight_mode = mode
        return True

    # --- telemetry -------------------------------------------------------------------------
    def _status(self, now: float, samples) -> None:
        for s in samples:
            if s.kind == "fcxy" and "hold_active" in s.data:
                self.fc_active, self.fc_fault, self.fc_mode = bool(s.data["hold_active"]), bool(s.data["hold_fault"]), int(s.data["hold_mode"]); self.fc_status_t = now
                if s.data.get("xy_disabled") and not self._xy_note:
                    self._xy_note = True; self.xy_active = "off"; self.events.append((now, "the FC sideways loop switched itself off (speed or distance kept growing)"))

    def observe(self, now: float, samples) -> None:
        super().observe(now, samples); self._status(now, samples)

    def preflight(self, now: float):
        ok, why = super().preflight(now)
        if now - self.fc_status_t > STATUS_MAX_AGE_S:
            why = list(why) + ["the FC sends no hold status (flash the fc-hybrid firmware, or the state frame is not arriving)"]
        ref = self.cfg.thr_hover0
        if not 0.3 <= ref <= 0.9:
            why = list(why) + [f"hover reference {ref:.2f} is outside 0.3..0.9"]
        return (not why), why

    # --- flight ----------------------------------------------------------------------------
    def start(self, now: float, force: bool = False) -> bool:
        ok = super().start(now, force)
        if ok:
            self._t_req = now; self._fallback = False; self._saw_land_mode = False; self._z_f = 0.0
            self.events.append((now, f"FC hold: target {self.cfg.hover_height:.2f} m, hover reference {self.cfg.thr_hover0:.2f}; the FC creeps up, lifts and holds by itself"))
        return ok

    def command_height(self, now: float, h: float) -> bool:
        if self.state not in (HoverState.TAKEOFF, HoverState.HOVER):
            return False
        self.cfg.hover_height = h; self.z_ref = h
        return True

    def land(self, now: float, why: str = "operator") -> None:
        if self.state in (HoverState.TAKEOFF, HoverState.SIGN_CHECK, HoverState.HOVER):
            self._set_state(now, HoverState.LAND, why)

    def hover_learned_flight(self, min_s: float = 3.0):
        return None                                           # the FC finds its own hover throttle; nothing to learn on the PC

    def _height(self, samples) -> Optional[float]:
        raw = None
        for s in samples:
            if s.kind == "optrange" and s.data.get("distance_m", 0.0) > 0.0:
                raw = s.data["distance_m"]
        if raw is None:
            return None
        return max(self._zrel(raw), 0.0)

    def update(self, now: float, samples, dt: float) -> Controls:
        c = self.cfg
        self._now = now; self._dt = dt
        if self.state in (HoverState.IDLE, HoverState.DONE):
            self.cmd = Controls(arm=False); return self.cmd
        for s in samples:
            self._ingest(now, s)                              # keeps the range history, rates and battery that preflight and the GCS read
        self._status(now, samples)
        h = self._height(samples)
        if h is not None:
            self._z_f += (h - self._z_f) * 0.5
        self.kz.x[0] = self._z_f; self.kz.x[1] = 0.0
        z = self._z_f
        self._zh.append((now, z)); self._zh = [p for p in self._zh if now - p[0] < 1.0]
        ref = throttle01_to_crsf(c.thr_hover0)
        fresh = now - self.fc_status_t < STATUS_MAX_AGE_S

        if self.state == HoverState.TAKEOFF:
            self.z_ref = c.hover_height
            # v4.7 firmware: the FC's hold already flies the climb from ~0.15 m (closed-loop take-off), so "hold mode" no longer means "at the height". HOVER only when
            # it is AT the target (within ARRIVED_M) and has been calm for 1 s: EXPLORE and the VO key frame must not start mid-climb. (v4.6: the hold began 5 cm below the target.)
            calm = len(self._zh) > 5 and now - self._zh[0][0] > 0.8 and max(p[1] for p in self._zh) - min(p[1] for p in self._zh) < 0.04
            holding = self.fc_active and self.fc_mode == 0 and z > LIFTED_M
            self._t_air = (self._t_air or now) if holding else None
            if holding and abs(z - c.hover_height) < ARRIVED_M and calm:
                self._set_state(now, HoverState.HOVER, "the FC lifted off and holds at the height")
            elif holding and now - self._t_air > AIR_GRACE_S:                      # airborne and holding but not within ARRIVED_M: still HOVER (never the take-off timeout in the air)
                self.events.append((now, f"holding at {z:.2f} m, not within {ARRIVED_M * 100:.0f} cm of {c.hover_height:.2f} m after {AIR_GRACE_S:.0f} s: HOVER anyway"))
                self._set_state(now, HoverState.HOVER, "the FC holds (not exactly at the height)")
            elif now - self._t_req > TAKEOFF_TIMEOUT_S and z < LIFTED_M:            # 2026-11-23 safety: 'no lift-off' (= disarm) ONLY while still on the floor
                self.events.append((now, f"no lift-off within {TAKEOFF_TIMEOUT_S:.0f} s of the request: stopped")); self._set_state(now, HoverState.DONE, "no lift-off")
                self.cmd = Controls(arm=False); return self.cmd
            elif not fresh and now - self._t_req > 3.0:
                self.events.append((now, "the FC hold status stopped arriving: stopped")); self._set_state(now, HoverState.DONE, "no FC status")
                self.cmd = Controls(arm=False); return self.cmd
        if self.state == HoverState.HOVER:
            self.z_ref = c.hover_height
            if z > c.ceiling and not self._fallback:
                self.events.append((now, f"height {z:.2f} m above the ceiling {c.ceiling:.2f} m: the FC lands slowly")); self._set_state(now, HoverState.LAND, f"ceiling {c.ceiling:.2f} m exceeded")
        if self.state in (HoverState.TAKEOFF, HoverState.HOVER, HoverState.LAND) and self.fc_fault and not self._fallback and self.state != HoverState.DONE:
            if z > 0.25:
                self._fallback = True; self.events.append((now, f"FC hold FAULT at {z:.2f} m: the PC sends a descending throttle until low"))
            else:
                self.events.append((now, f"FC hold FAULT at {z:.2f} m: stopped")); self._set_state(now, HoverState.DONE, "FC hold fault")
                self.cmd = Controls(arm=False); return self.cmd
        if self._fallback:
            if z < 0.25:
                self._set_state(now, HoverState.DONE, "FC hold fault: low enough")
                self.cmd = Controls(arm=False); return self.cmd
            self.cmd = Controls(arm=True, throttle=0.9 * c.thr_hover0, aux5=CRSF_MIN, aux6=ref, aux7=mode_to_aux7(self.flight_mode)); return self.cmd
        if self.state == HoverState.LAND:
            if self.fc_mode == 2 and self.fc_active: self._saw_land_mode = True
            if not self.fc_active and z < 0.12 and (self._saw_land_mode or now - self.t_state > 8.0):
                self._set_state(now, HoverState.DONE, "landed")
                self.cmd = Controls(arm=False); return self.cmd
            self.cmd = Controls(arm=True, aux5=AUX5_LAND, aux6=ref, aux7=mode_to_aux7(self.flight_mode)); return self.cmd
        self.cmd = Controls(arm=True, aux5=height_to_aux5(c.hover_height), aux6=ref, aux7=mode_to_aux7(self.flight_mode))       # throttle stick stays at minimum: the FC owns the throttle
        return self.cmd
