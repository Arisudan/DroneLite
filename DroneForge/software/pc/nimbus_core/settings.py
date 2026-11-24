"""Persistent flight settings (JSON), validated. The values that must come from MEASUREMENTS on the real drone live here:
hover throttle (from a real lift-off) and the hover height. Used by HoverAutonomy; edited in the GCS Settings panel."""
from __future__ import annotations

import json
import os
from dataclasses import asdict, dataclass, fields, replace

PATH = os.environ.get("DRONEFORGE_SETTINGS", os.path.expanduser("~/Music/DroneForge/config/flight_settings.json"))

LIMITS = {"hover_height": (0.20, 2.50), "ceiling_m": (0.3, 3.0), "thr_hover0": (0.15, 0.88), "thr_max": (0.25, 1.00), "batt_ref_v": (12.0, 17.0), "creep_per_s": (0.005, 0.05), "battery_scale": (0.5, 2.0)}

HEADROOM = 1.05            # thr_max must exceed the hover throttle by this factor (was 1.15: boxes need 0.75 with the cap at 0.81)
LEARN_ALPHA, LEARN_MAX_STEP, LEARN_RANGE = 0.3, 0.08, (0.30, 0.85)

# Flight profile for THIS drone (4S, 3", 382 g): found by simulation against the measured plant and sensor behaviour (docs/status_logs/flight-profile_test-history-and-findings.md).
PROFILE = dict(kp_z=0.9923, kv_z=0.08, ki_z=0.01, i_max=0.078, sensor_delay_s=0.1118, bias_ff=0.85, vz_lim=0.4581, takeoff_rate=0.3646, takeoff_ramp_rate=0.03,
               takeoff_ramp_slow=0.0074, takeoff_slow_frac=0.953, takeoff_lift_margin=0.0, thrust_exp=2.0, range_untrusted=(0.07, 0.40), range_untrusted_r_scale=4.0, range_over_read_m=0.0, ge_bonus=0.25, climb_overspeed_gain=0.1, ge_h_m=0.1,
               lift_blend_s=1.5, terrain_gate=True, thr_slew_per_s=0.5, kv_z_climb=0.15, overshoot_soft=0.5, lift_ref_ge=True,
               # 2026-11-11 automatic take-off: takeoff_lift_margin -0.05 -> 0 (no effect: the climb velocity loop's integrator starts at the lift-off throttle and finds the hover throttle on any surface),
               # ground-referenced estimation (range_untrusted and range_cal are applied to the height above the learned ground reading), adaptive creep (0.0074 -> 0.02 /s without motion), timeout from throttle cap
               # 2026-11-10 vertical hold fix (sim, not flown): bias_ff 0.6135 -> 0.85 (the hover-throttle error after lift-off left a 5-6 cm hold offset), climb_overspeed_gain 0.3 -> 0.1 plus a
               # symmetric climb velocity gain (the one-sided brake near the target dropped the drone 0.2 m), the hover reference from the lift-off throttle THROUGH the ground-effect model
               # (a fixed -0.05 margin is 0.01-0.02 too high: the drone creeps up to 5 cm in ground effect before the lift detector fires), a proportional overshoot damper and a 0.5/s throttle slew
               # (the hard -0.08 step dropped the drone into ground effect), the terrain gate never absorbs within 10 s of hover entry or while moving and follows the sensor when the
               # disagreement is not a clean floor step (it used to leave the estimate 0.3 m behind a real descent), range_untrusted_r_scale 60 -> 4 with the over-read corrected by range_cal
               # the over-read is CORRECTED (reading -> true distance, from the measured curve below), and only mildly distrusted (x4 = sigma 2x) so the height estimate stays
               # observable through the 7-40 cm band: with x60 it coasted blind on the throttle model, which is wrong by the ground-effect bonus and the lift-off margin there
               range_cal=((0.06, 0.06), (0.29, 0.20), (0.41, 0.32), (0.45, 0.45)))
# 2026-11-13 REAL profile, fitted to the real flights of 11-12/11-13 (docs/maintenance/REAL_FLIGHT_FIT_2026-11-13.md): the throttle-to-range delay is 0.4-0.55 s (the legacy sim had 0.2),
# the free-air hover is 0.69-0.75. The estimator models the command delay (thr_delay_s), the height loop controls on the state PREDICTED that far ahead (pred_states), the bias observer
# is off (it took lag for model error and amplified it), the velocity gains are lower, the sign check is off (it never ran in a real flight), and the throttle pinned at the cap
# without a climb lands the drone (cap_land_s). The LEGACY profile above stays the default so the older sim tests keep their meaning.
PROFILE_REAL = dict(PROFILE, kp_z=0.6, kv_z=0.07, ki_z=0.015, bias_ff=0.0, thr_delay_s=0.3, pred_states="air+land", sign_check=False, cap_land_s=1.5, land_kv=0.05)
SENSOR_GROUND_M = 0.07                         # what the range sensor reads with the drone sitting on the floor (real flight log: 0.07-0.08 m)
SENSOR_LOW_BIAS_M = 0.09                       # the MTF-02P reads ~9 cm too HIGH between about 20 and 35 cm (measured); accurate from 50 cm: hold at >= 0.5 m. PROFILE['range_cal'] is the inverse of this curve


@dataclass
class FlightSettings:
    hover_height: float = 0.5        # m (first tethered flight: 0.3)
    thr_hover0: float = 0.42         # stick 0..1 at full battery: UNMEASURED default until the ramp tool has run
    thr_max: float = 0.70            # hard cap on the throttle the hover controller may command
    measured: bool = False           # True once thr_hover0 came from a real ramp
    batt_ref_v: float = 15.2         # pack voltage at which thr_hover0 was measured (the controller scales the throttle with the voltage relative to this)
    creep_per_s: float = 0.0074      # throttle per second while the drone is creeping up to lift-off (stick fraction/s); lift-off takes (lift - 0.95*hover)/creep seconds
    xy_source: str = "fc"            # horizontal estimate: "fc" = the FC EKF when it agrees with the flow (else the PC flow filter), "pc" = always the PC flow filter
    battery_scale: float = 1.0       # display-only correction: shown pack voltage = FC reading x this (the controllers keep the raw reading)
    ceiling_m: float = 3.0           # tether length / hard height limit: the safety layer lands above it and no height above it can be set
    profile: str = "legacy"          # height loop: "legacy" PC loop (older tuning), "real" PC loop fitted to the real flights (0.4-0.55 s delay, fragile), "fc" the FC holds the height (hybrid)

    def validate(self) -> list[str]:
        bad = [f"{k}={getattr(self, k)} outside {lo}..{hi}" for k, (lo, hi) in LIMITS.items() if not lo <= getattr(self, k) <= hi]
        if self.profile not in ("legacy", "real", "fc"):
            bad.append(f"profile {self.profile!r} must be 'legacy', 'real' or 'fc'")
        if self.xy_source not in ("fc", "pc"):
            bad.append(f"xy_source {self.xy_source!r} must be 'fc' or 'pc'")
        if self.hover_height > self.ceiling_m:
            bad.append(f"hover_height {self.hover_height} m is above the ceiling {self.ceiling_m} m")
        if self.thr_max < self.thr_hover0 * HEADROOM:
            bad.append(f"thr_max {self.thr_max} must be at least {HEADROOM} x thr_hover0 ({self.thr_hover0 * HEADROOM:.2f}) or the controller cannot climb")
        return bad


def load(path: str = PATH) -> FlightSettings:
    try:
        d = json.load(open(path))
        kw = {k: d[k] for k in asdict(FlightSettings()) if k in d}
        s = FlightSettings(**kw)
        return s if not s.validate() else FlightSettings()
    except (OSError, ValueError, TypeError):
        return FlightSettings()


def save(s: FlightSettings, path: str = PATH) -> None:
    bad = s.validate()
    if bad:
        raise ValueError("; ".join(bad))
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    json.dump(asdict(s), open(tmp, "w"), indent=1)
    os.replace(tmp, path)


def learn_hover(s: FlightSettings, norm_hover: float):
    """Exponential moving average (alpha 0.3) of the measured hover throttle (normalised to batt_ref_v, see HoverController.hover_learned_flight) into thr_hover0:
    at most +/-0.08 per flight, bounded to [0.30, 0.85] and so that thr_max keeps its headroom. Returns (new settings, event text) or None when nothing changes.
    measured, batt_ref_v and everything else stay as they are."""
    old = s.thr_hover0
    step = min(max(LEARN_ALPHA * (norm_hover - old), -LEARN_MAX_STEP), LEARN_MAX_STEP)
    new = min(max(old + step, LEARN_RANGE[0]), LEARN_RANGE[1], s.thr_max / HEADROOM)
    new = round(new, 4)
    if abs(new - old) < 5e-4:
        return None
    return replace(s, thr_hover0=new), f"learned hover throttle {new:.2f} (was {old:.2f})"


def apply_to_config(s: FlightSettings, cfg) -> None:
    cfg.hover_height, cfg.thr_hover0, cfg.thr_max, cfg.batt_v0 = s.hover_height, s.thr_hover0, s.thr_max, s.batt_ref_v
    for k, v in (PROFILE_REAL if s.profile == "real" else PROFILE).items():
        setattr(cfg, k, v)
    cfg.takeoff_ramp_slow = s.creep_per_s
    cfg.xy_source = s.xy_source
    cfg.sign_height = min(s.hover_height, 0.5)              # sign check at the hold height: the range sensor is accurate there, and not in the 20-35 cm zone
    cfg.ceiling = max(cfg.ceiling, s.hover_height + 0.3)


def plant_params(s: FlightSettings):
    """Simulator plant that matches this drone's measured numbers (hover throttle, quadratic thrust, pack voltage, sensor over-read)."""
    from nimbus_pc.hover.sim import SimParams
    real = dict(ctrl_delay_s=0.33, optrange_hz=17.0, tau_motor=0.06) if s.profile == "real" else {}      # the real drone's delay and the compact-frame rate (bench 17.9 Hz)
    return SimParams(**real, hover_thr=s.thr_hover0, thrust_exp=2.0, batt_v0=s.batt_ref_v, batt_sag_per_s=0.005, range_low_bias_m=SENSOR_LOW_BIAS_M, ground_range_m=SENSOR_GROUND_M,
                     ground_effect_bonus=0.25, ground_effect_h_m=0.1)   # measured 2026-11-10: leaves the floor at ~0.62 (ground effect) but needs ~0.68 to climb


# Field dict keys: key, label, unit, kind ('number'|'choice'), min, max, step, choices [(value, text)], tooltip, experimental (whole field),
# experimental_values (choice values to label EXPERIMENTAL), readonly (+ value). Number fields: min/max/step; choice fields: choices.
# --- declarative schema: the GUI builds its settings forms from this (no widget knowledge here) ---------------------------------
def _num(key, label, unit, lo, hi, step, tip, **kw):
    return dict(key=key, label=label, unit=unit, kind="number", min=lo, max=hi, step=step, choices=None, tooltip=tip, experimental=False, experimental_values=[], readonly=False, **kw)


def _choice(key, label, choices, tip, experimental_values=()):
    """choices: list of (value, text); a choice whose value is in `experimental_values` is shown with an 'experimental' label."""
    return dict(key=key, label=label, unit="", kind="choice", min=None, max=None, step=None, choices=choices, tooltip=tip, experimental=False, experimental_values=list(experimental_values), readonly=False)


def _ro(key, label, unit, value, tip):
    return dict(key=key, label=label, unit=unit, kind="number", min=value, max=value, step=None, choices=None, tooltip=tip, experimental=False, experimental_values=[], readonly=True, value=value)


SCHEMA = [
    {"group": "Flight", "fields": [
        _num("hover_height", "Hover height", "m", *LIMITS["hover_height"], 0.05, "Height held after take-off (range-sensor height). Must not exceed the ceiling."),
    ]},
    {"group": "Safety", "fields": [
        _num("ceiling_m", "Ceiling (tether length)", "m", *LIMITS["ceiling_m"], 0.1, "Hard height limit: the drone lands above it and no higher hover height can be set. Set it to the tether length."),
        _ro("tilt_sustained_deg", "Disarm tilt (sustained)", "deg", 45.0, "Fixed by the safety layer: disarm if tilted this far for 0.5 s."),
        _ro("tilt_extreme_deg", "Disarm tilt (instant)", "deg", 70.0, "Fixed by the safety layer: instant disarm."),
    ]},
    {"group": "Controller", "fields": [
        _num("thr_hover0", "Hover throttle", "", *LIMITS["thr_hover0"], 0.005, "Stick fraction (0..1) that hovers at the reference voltage. Measured by the throttle ramp."),
        _choice("measured", "Hover throttle status", [(False, "Not measured (take-off is refused)"), (True, "Measured (take-off allowed)")],
                "Take-off on the real drone is refused until this says Measured. Set it only after the hover throttle and the reference voltage came from a real lift-off (a ramp or a log): hover throttle and reference voltage are the values at which the drone lifted off."),
        _num("thr_max", "Throttle cap", "", *LIMITS["thr_max"], 0.01, "Hard cap on the throttle the controller may command (at least 1.15 x hover throttle)."),
        _num("creep_per_s", "Take-off creep", "/s", *LIMITS["creep_per_s"], 0.001, "Throttle added per second while creeping up to lift-off. Smaller = gentler but slower: the creep speeds up by itself (to 0.02/s) while nothing moves; the take-off stops 3 s after the throttle reaches the cap or after 30 s without lift-off."),
    ]},
    {"group": "Battery", "fields": [
        _num("batt_ref_v", "Reference voltage", "V", *LIMITS["batt_ref_v"], 0.1, "Pack voltage at which the hover throttle was measured; the controller scales the throttle with voltage relative to this (hover scaling)."),
        _num("battery_scale", "Voltage correction factor", "x", *LIMITS["battery_scale"], 0.01, "Multiplies the FC's pack-voltage reading for DISPLAY only (voltage and percentage); the flight controllers keep the raw reading. Use the Calibrate button below with a fully charged pack."),
    ]},
    {"group": "Height loop", "fields": [
        _choice("profile", "Who holds the height", [("legacy", "PC loop, old tuning"), ("real", "PC loop, real-flight fit (fragile)"), ("fc", "Flight controller (hybrid)")],
                "Flight controller (hybrid): the FC creeps up, lifts, holds and lands by itself (needs the fc-hybrid firmware); the PC only requests it and watches. The PC loops fight a 0.4-0.55 s radio delay."),
    ]},
    {"group": "Estimation", "fields": [
        _choice("xy_source", "Horizontal position source", [("fc", "Flight controller EKF"), ("pc", "PC flow filter")],
                "FC: the FC EKF when it agrees with the optical flow (else the PC filter). PC: always the PC flow filter."),
    ]},
]
FIELD_KEYS = [f["key"] for g in SCHEMA for f in g["fields"] if not f["readonly"]]
