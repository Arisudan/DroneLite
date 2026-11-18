"""Scenario runner: drives DroneSim with HoverController at 100 Hz like the real sender, and reports metrics."""
from __future__ import annotations

import csv
import math
from dataclasses import dataclass, field, replace
from typing import Optional

from .controller import HoverConfig, HoverController, HoverState
from nimbus_core.safety import SafetyMonitor, SafetyConfig

from .sim import DroneSim, SimParams

SAFETY_SCENARIOS = {"low_battery", "ceiling"}          # the safety monitor is wired in for these (it must not change the other scenarios' results)

# controller settings of the real drone (flight profile) so the scenario shows what the real flight would do
_GE_CFG = {"thr_hover0": 0.68, "kp_z": 0.9923, "ki_z": 0.01, "i_max": 0.078, "sensor_delay_s": 0.1118, "bias_ff": 0.6135, "vz_lim": 0.4581,
           "takeoff_rate": 0.3646, "takeoff_ramp_rate": 0.03, "takeoff_ramp_slow": 0.0074, "takeoff_slow_frac": 0.953, "thrust_exp": 2.0, "lift_blend_s": 1.5,
           "kv_z": 0.12, "sign_check": False}      # kv_z raised and the sign check off: the sign check at 0.25 m is flaky while the drone bounces out of ground effect (separate issue)

# the saved flight profile (settings.PROFILE) on the measured real drone: lift ~0.61, hover ~0.68, ground effect to 0.1 m, sensor reads 7 cm on the floor and over-reads below 35 cm
from nimbus_core.settings import PROFILE as _PROFILE
_GE_PLANT = {"hover_thr": 0.68, "thrust_exp": 2.0, "batt_v0": 15.2, "batt_sag_per_s": 0.005, "range_low_bias_m": 0.09, "ground_range_m": 0.07, "ground_effect_bonus": 0.25, "ground_effect_h_m": 0.1}
_PROFILE_CFG = dict(_PROFILE, thr_hover0=0.68, thr_max=0.80, batt_v0=15.2, sign_check=False)

SCENARIOS = {
    "nominal": ("Ideal-ish: noise, delay, drops as measured on the real link", {}, {}),
    "heavy": ("Drone 15 % heavier than assumed, battery sags (hover throttle is wrong)", {"mass_factor": 1.15}, {}),
    "light": ("Drone 12 % lighter than assumed", {"mass_factor": 0.88}, {}),
    "wind": ("Two gusts: 0.8 m/s^2 sideways for 1 s (in the hold: the takeoff is slow and careful now)", {"wind": [(19.0, 20.0, 0.0, 0.8), (24.0, 25.0, 0.6, 0.0)]}, {}),
    "slow_telemetry": ("Telemetry at the default ratio: range/flow only 4 Hz, 200-300 ms old", {"optrange_hz": 4.0, "sensor_delay": (0.2, 0.3)}, {"sensor_delay_s": 0.25}),
    "dropout": ("0.8 s with no telemetry while hovering", {"gaps": [(14.0, 14.8)]}, {}),
    "blackout": ("Telemetry lost for good at 14 s", {"gaps": [(14.0, 99.0)]}, {}),
    "low_battery": ("Pack drains fast (0.07 V/s) with the OPTIONAL auto-landing on: the safety layer must make the controller land before it is empty", {"batt_sag_per_s": 0.07}, {"batt_autoland": True}),
    "ceiling": ("Hover height set above the 2.5 m indoor limit: the ceiling guard must make it land", {}, {"hover_height": 3.0}),
    "ground_effect": ("Real-drone lift behaviour: ground effect lifts it to ~5 cm at 0.62 throttle but it needs the hover throttle (0.68) to climb", {"hover_thr": 0.68, "ground_effect_bonus": 0.22, "ground_effect_h_m": 0.1, "thrust_exp": 2.0}, _GE_CFG),
    "range_dropouts": ("ground_effect plus 20 % of range samples reading the floor (0.01 m) while airborne", {"hover_thr": 0.68, "ground_effect_bonus": 0.22, "ground_effect_h_m": 0.1, "range_dropout_p": 0.2, "thrust_exp": 2.0, "sensor_drop": 0.0}, _GE_CFG),
    "lean_drift": ("Real-drone ground effect plus a constant 0.8 deg level offset (0.14 m/s^2 sideways) the whole flight: the drone must stay at its take-off point", _GE_PLANT | {"wind": [(0.0, 999.0, 0.1, 0.1)]}, _PROFILE_CFG),
    "phantom_flow": ("Real-drone ground effect plus heavy-tailed flow spikes (5 %, 1-2 m/s, quality 40-130) and 0.1 m/s flow noise below 0.3 m", _GE_PLANT | {"flow_spike_p": 0.05, "flow_noise_low": 0.1}, _PROFILE_CFG),
    "sign_error": ("Pitch/roll wired with the wrong sign (the classic first-flight mistake)", {"pitch_sign": -1.0, "roll_sign": -1.0}, {}),
}


@dataclass
class Result:
    name: str
    rows: list = field(default_factory=list)
    metrics: dict = field(default_factory=dict)
    events: list = field(default_factory=list)


def run(name: str = "nominal", duration: float = 40.0, seed: int = 1, land_at: Optional[float] = 30.0, hz: float = 100.0,
        params: Optional[SimParams] = None, cfg: Optional[HoverConfig] = None, height_steps: Optional[list] = None, mode_steps: Optional[list] = None) -> Result:
    desc, sim_over, cfg_over = SCENARIOS[name]
    p = replace(params or SimParams(), **sim_over)
    c = replace(cfg or HoverConfig(), **cfg_over)
    sim, ctl = DroneSim(p, seed), HoverController(c)
    mon = SafetyMonitor(SafetyConfig(batt_autoland=True)) if name in SAFETY_SCENARIOS else None        # the optional auto-landing is what these scenarios test
    mon_armed_t = None
    dt = 1.0 / hz
    res = Result(name)
    t_start = 3.5                                  # let the telemetry run for a few seconds so the preflight can judge it
    t_hover = None
    hover_xy = (0.0, 0.0)
    hold_from = None
    landed_t = None
    max_tilt = 0.0
    max_z_before_hover = 0.0
    t_lift = t_02 = t_05 = None
    n = int(duration / dt)
    for i in range(n):
        t = sim.t
        if ctl.state == HoverState.IDLE and t >= t_start and not ctl.refused:
            if not ctl.start(t):
                res.events = list(ctl.events); res.metrics = {"scenario": name, "description": desc, "refused": list(ctl.refused), "reached_hover": False,
                                                              "crashed": False, "final_state": "idle", "end_reason": "takeoff refused: " + "; ".join(ctl.refused),
                                                              "touchdown_speed_mps": 0.0, "max_tilt_deg": 0.0, "final_z_m": 0.0, "final_xy_m": 0.0,
                                                              "t_hover_s": None, "alt_err_rms_m": None, "alt_err_max_m": None, "xy_drift_max_m": None, "overshoot_m": None}
                return res
        if land_at is not None and t >= land_at and ctl.state in (HoverState.TAKEOFF, HoverState.SIGN_CHECK, HoverState.HOVER):
            ctl.land(t, "scheduled")
        for hs in (height_steps or []):                 # [(time_s, new_height_m)]: height commands in flight, like the GCS dropdown
            if t >= hs[0] and len(hs) == 2:
                ctl.command_height(t, hs[1]); hs.append(True)
        for ms in (mode_steps or []):                   # [(time_s, mode)]: flight-mode switches in flight, like the GCS selector
            if t >= ms[0] and len(ms) == 2:
                ctl.set_flight_mode(ms[1]); ms.append(True)
        samples = sim.poll()
        if mon is not None:
            for smp in samples:
                if smp.kind == "battery": mon.feed_battery(t, smp.data["voltage_v"])
                elif smp.kind == "optrange": mon.feed_range(t, smp.data["distance_m"])
                elif smp.kind == "attitude": mon.feed_attitude(t, smp.data["pitch_deg"], smp.data["roll_deg"])
            mon.feed_link(t, 100.0)
            for ev in mon.check(t, armed=sim.armed):
                res.events.append((t, f"SAFETY [{ev.level}] {ev.text}"))
            if mon.land_request and ctl.state in (HoverState.TAKEOFF, HoverState.SIGN_CHECK, HoverState.HOVER):
                ctl.land(t, "safety: " + mon.land_request)
        if ctl.state == HoverState.IDLE:
            ctl.observe(t, samples)                    # judge the telemetry before takeoff
            samples = []
        cmd = ctl.update(t, samples, dt)
        sim.step(dt, cmd.roll, cmd.pitch, cmd.throttle, cmd.arm)
        if t_hover is None and ctl.state == HoverState.HOVER:
            t_hover = t
            hover_xy = (sim.x, sim.y)
        if t_hover is None and ctl.state in (HoverState.SIGN_CHECK,) and hold_from is None:
            hold_from = t
        max_tilt = max(max_tilt, math.degrees(max(abs(sim.tilt_x), abs(sim.tilt_y))))
        if t_hover is None:
            max_z_before_hover = max(max_z_before_hover, sim.z)
        if t_lift is None and sim.z > 0.02: t_lift = t
        if t_lift is not None and t_02 is None and sim.z >= 0.2: t_02 = t
        if t_lift is not None and t_05 is None and sim.z >= 0.5: t_05 = t
        if i % int(hz / 10) == 0:
            res.rows.append({"t": round(t, 2), "state": ctl.state.value, "z": sim.z, "z_est": ctl.z, "vz": sim.vz, "x": sim.x, "y": sim.y,
                             "tilt_deg": math.degrees(max(abs(sim.tilt_x), abs(sim.tilt_y))), "cmd_tilt_deg": c.angle_limit_deg * max(abs(cmd.roll), abs(cmd.pitch)), "thr": cmd.throttle, "batt": sim.batt_v})
        if ctl.state == HoverState.DONE and landed_t is None:
            landed_t = t
        if sim.crashed or (landed_t is not None and t - landed_t > 1.5):
            break
    res.events = list(ctl.events) + [e for e in res.events]
    hold = [r for r in res.rows if t_hover is not None and r["t"] >= t_hover + 4.0 and (land_at is None or r["t"] <= land_at)]
    h = c.hover_height
    res.metrics = {
        "scenario": name, "description": desc,
        "reached_hover": t_hover is not None, "t_hover_s": t_hover,
        "alt_err_rms_m": math.sqrt(sum((r["z"] - h) ** 2 for r in hold) / len(hold)) if hold else None,
        "alt_err_max_m": max((abs(r["z"] - h) for r in hold), default=None),
        "xy_drift_max_m": max((math.hypot(r["x"] - hover_xy[0], r["y"] - hover_xy[1]) for r in hold), default=None),
        "overshoot_m": max(max_z_before_hover - h, 0.0) if t_hover is not None else None, "max_tilt_deg": max_tilt, "touchdown_speed_mps": sim.max_impact, "crashed": sim.crashed,
        "t_lift_s": t_lift, "lift_to_0p2_s": (t_02 - t_lift) if (t_lift and t_02) else None, "lift_to_0p5_s": (t_05 - t_lift) if (t_lift and t_05) else None,
        "xy_from_origin_max_m": max((math.hypot(r["x"], r["y"]) for r in res.rows if t_hover is not None and r["t"] <= t_hover + 10.0), default=None),
        "cmd_tilt_below_0p2_max_deg": max((r["cmd_tilt_deg"] for r in res.rows if r["z"] < 0.2), default=0.0),
        "final_state": ctl.state.value, "final_z_m": sim.z, "final_xy_m": math.hypot(sim.x, sim.y),
        "end_reason": ctl.reason,
    }
    return res


def write_csv(res: Result, path: str) -> None:
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(res.rows[0].keys()))
        w.writeheader(); w.writerows(res.rows)


def summary(res: Result) -> str:
    m = res.metrics
    f = lambda v, u="": "--" if v is None else (f"{v:.3f}{u}" if isinstance(v, float) else str(v))
    lines = [f"[{m['scenario']}] {m['description']}",
             f"  hover reached: {m['reached_hover']} at {f(m['t_hover_s'], ' s')} (takeoff overshoot {f(m['overshoot_m'], ' m')}) | altitude error rms {f(m['alt_err_rms_m'], ' m')} max {f(m['alt_err_max_m'], ' m')} "
             f"| xy drift max {f(m['xy_drift_max_m'], ' m')}",
             f"  max tilt {f(m['max_tilt_deg'], ' deg')} | touchdown {f(m['touchdown_speed_mps'], ' m/s')} | crashed {m['crashed']} | end: {m['final_state']} "
             f"({m['end_reason'] or '-'}) z {f(m['final_z_m'], ' m')}",
             "  events: " + "; ".join(f"{t:.1f}s {e}" for t, e in res.events)]
    return "\n".join(lines)
