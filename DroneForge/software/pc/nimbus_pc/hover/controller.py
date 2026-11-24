"""Hover hold controller: altitude from the range sensor, horizontal position from optical-flow velocity, on top of Betaflight ANGLE mode.

Designed for the data we measured on the real link: range/flow ~10 Hz, attitude ~3.6 Hz, 80-150 ms old, with gaps. So every
measurement goes through a Kalman filter that predicts between samples from the commands we sent, and the control gains are low.
Body-rate damping (Nimbus has it) needs high-rate gyro data, which the 7 Hz link does not give: it is NOT faked here, the velocity damping (kv_xy) stays the only damping term.
Vertical references (take-off climb, height changes, landing descent) come from the jerk-limited S-curve generator in trajectory.py.
All numbers are first guesses to be tuned in the simulator and then on a tethered flight (UNVERIFIED on the real drone).
Output is a nimbus_pc Controls (roll/pitch -1..1 stick, throttle 0..1, arm), always clamped by the safety envelope below.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from enum import Enum
from typing import Optional

from nimbus_pc.sender import Controls
from .trajectory import JerkLimited1D

G = 9.81


class HoverState(Enum):
    IDLE = "idle"
    TAKEOFF = "takeoff"
    SIGN_CHECK = "sign_check"
    HOVER = "hover"
    LAND = "land"
    SENSOR_LOSS = "sensor_loss"
    DONE = "done"


@dataclass
class HoverConfig:
    hover_height: float = 0.5
    takeoff_rate: float = 0.15
    land_rate: float = 0.10
    land_cut_height: float = 0.03
    land_slow_below: float = 0.15       # (unused since the Nimbus-style landing: constant land_rate down to land_ramp_height)
    land_min_rate: float = 0.05
    land_rate_high: float = 0.30        # planned descent speed above land_slow_height (below it land_rate)
    land_slow_height: float = 0.60
    land_vz_cap: float = 0.50           # measured descent speed cap: throttle is raised in proportion above it
    land_cap_gain: float = 1.0
    land_cap_max: float = 0.15
    land_brake_gain: float = 0.15
    land_brake_max: float = 0.03
    land_kv: float = 0.08               # descent-rate P gain (throttle per m/s)
    land_ki: float = 0.0
    land_i_max: float = 0.06
    land_floor_frac: float = 0.85       # lowest throttle during the descent, as a fraction of the hover throttle (until land_ramp_height)
    land_touch_frac: float = 0.92
    land_touch_hold_s: float = 0.0
    land_contact_ramp_s: float = 0.6    # on contact the throttle ramps down for this long before the disarm
    land_ramp_height: float = 0.04      # landing: below this height above the floor reading the throttle ramps to idle
    land_arm_zref: float = 0.15         # the touchdown ramp may start only once the planned descent height is below this
    land_ramp_s: float = 1.0            # ... linearly over this long, then DONE (the caller disarms)
    land_contact_height: float = 0.02   # below this for land_contact_s while landing = on the floor: finish at once
    land_contact_s: float = 0.2
    land_sink_guard: float = 0.15       # during the ramp, if sinking faster than this (m/s) the altitude loop throttle is kept as a lower bound (soft touchdown)
    ref_v_max: float = 0.25             # jerk-limited vertical reference (Nimbus: 0.25 m/s, 0.35 m/s^2, 0.8 m/s^3)
    ref_a_max: float = 0.35
    ref_j_max: float = 0.8
    lift_grace_s: float = 1.5           # after lift-off the overshoot damper's height margin is widened by lift_grace_m for this long
    lift_grace_m: float = 0.15
    hover_learn_min_s: float = 3.0      # settled hover seconds before the measured hover throttle replaces the lift-off reference
    hover_learn_report_s: float = 5.0   # settled hover seconds before hover_learned() reports it
    ceiling: float = 1.2
    # throttle model (stick 0..1). thr_hover0 is the full-battery hover throttle: UNKNOWN on the real drone, calibrate first
    thr_hover0: float = 0.42
    thrust_exp: float = 1.0              # thrust ~ (throttle above idle)^exp. Real props/motors: about 2 (set from the flight settings); 1 = old linear model
    thr_idle: float = 0.05
    thr_min_air: float = 0.15
    thr_max: float = 0.70
    overshoot_m: float = 0.12            # above the height target by this much (or climbing faster than overshoot_vz) -> pull throttle below hover at once
    overshoot_vz: float = 0.45
    overshoot_cut: float = 0.08          # how far below the hover throttle the overshoot damper goes
    batt_v0: float = 16.2
    # altitude loop
    kp_z: float = 1.0
    kv_z: float = 0.10
    ki_z: float = 0.03
    bias_ff: float = 1.0                 # fraction of the learned acceleration bias fed back as throttle (a disturbance observer)
    min_rate_hz: float = 6.5             # preflight: range/flow samples per second needed to fly at all
    max_gap_s: float = 0.4               # preflight: longest allowed gap between range samples
    min_rate_inflight_hz: float = 5.0    # in flight: below this for 2 s -> land
    i_max: float = 0.10
    vz_lim: float = 0.35
    # horizontal loop (flow velocity)
    kp_pos: float = 0.50
    kv_xy: float = 1.5
    ki_xy: float = 0.15                  # position integral (m/s^2 per m*s), conditional anti-windup; contribution limited to xy_i_max_deg of tilt
    xy_i_max_deg: float = 3.0
    xy_i_range_m: float = 0.5            # integrate only while the position error is below this
    xy_min_height_m: float = 0.20        # XY hold acts only above this estimated height (the flow sensor is useless this close to the floor)
    xy_height_hyst_m: float = 0.05       # ...and closes again below xy_min_height_m - this
    xy_min_flow_q: int = 60              # flow quality (0-255) needed for XY hold
    xy_max_speed_ms: float = 1.0         # horizontal speed above this is a flow artefact: gate closes
    xy_speed_hold_s: float = 0.5         # ...and stays closed at least this long
    xy_flow_max_age_s: float = 0.5       # flow data older than this is not fresh
    xy_damp_min_height_m: float = 0.06   # from this estimated height only the velocity feedback acts (no position hold, no integral), limited to xy_damp_tilt_max_deg
    xy_damp_hyst_m: float = 0.04         # ...and it closes again below xy_damp_min_height_m - this
    xy_damp_tilt_max_deg: float = 3.0
    kv_xy_damp: float = 2.0              # velocity gain in the damping-only zone (stiffer than kv_xy: it must hold a ~0.8 deg level offset with almost no drift)
    xy_damp_ki: float = 2.0              # damping zone: a constant sideways push (FC level offset, ground effect) is learned from the velocity (m/s^2 per m/s per s) and handed to the position integral when the full hold opens
    xy_blend_s: float = 1.0              # damping -> full hold: the position term and the tilt authority fade in over this long
    xy_slew_deg_s: float = 10.0          # tilt command change limit while damping / blending (1 deg per 100 ms)
    xy_median_n: int = 5                 # velocity input = median of this many flow / FC samples (heavy-tailed spikes near the floor)
    xy_disagree_ms: float = 0.4          # FC EKF and PC flow differ by more than this: use the one with the smaller magnitude
    xy_origin_max_m: float = 0.3         # the hold reference is the take-off point if the estimate since lift-off moved less than this
    v_lim: float = 0.5
    tilt_max_deg: float = 8.0
    angle_limit_deg: float = 60.0        # Betaflight angle_limit: stick 1.0 = this many degrees (verify on the bench)
    pitch_sign: float = 1.0              # +1: positive pitch stick = accelerate forward. VERIFY with a bench tilt test before flying
    roll_sign: float = 1.0
    # plant model used for prediction
    tau_motor: float = 0.06
    pred_states: str = ""                # '' = off; 'air' = control on the state PREDICTED thr_delay_s ahead (Smith-style lead from the commands still in flight) in take-off and hover
    pred_gain: float = 1.0               # fraction of the predicted lead that is used (1 = full; less = robust against a delay that differs from thr_delay_s)
    cap_land_s: float = 0.0              # >0: airborne with the throttle pinned at the cap for this long while NOT climbing = the thrust limit is reached: land gently instead of fighting (a stronger demand cannot help)
    thr_delay_s: float = 0.0             # the throttle takes this long to reach the thrust (PC -> radio -> FC -> RC smoothing -> ESC): the estimator's model uses the command from this long ago (real flights 2026-11-12/13: total throttle-to-range delay 0.4-0.55 s)
    tau_att: float = 0.08
    drag: float = 0.3
    sensor_delay_s: float = 0.12
    # safety envelope
    stale_s: float = 0.5
    sensor_loss_open_loop_s: float = 1.0
    sensor_loss_timeout_s: float = 3.5   # blind descent ends on this timer (the estimate cannot be trusted without sensors)
    sign_check: bool = True
    sign_height: float = 0.25            # the sign check runs this low, with horizontal control off, before climbing to hover_height
    sign_window_hi: float = 0.10         # the sign check may start only for z in [sign_height - 0.03, sign_height + this] ...
    sign_max_vz: float = 0.1             # ... and |vz| below this; otherwise the take-off holds at sign_height (or skips the check when it climbed past the window)
    sign_pulse_deg: float = 2.0
    sign_min_response_mps: float = 0.08
    sensor_loss_thr_drop: float = 0.02   # throttle below hover during open-loop descent
    tilt_abort_deg: float = 35.0
    runaway_m: float = 0.6
    runaway_speed: float = 0.7
    runaway_hold_s: float = 0.8          # beyond runaway_m / runaway_speed for this long (a single gust is fine)
    batt_autoland: bool = False          # False (default): a low battery only WARNS; True: the controller lands by itself below batt_land_v
    batt_land_v: float = 13.2
    batt_warn_v: float = 14.0
    flow_min_quality: int = 60
    xy_source: str = "fc"                # horizontal speed/position from: "fc" = the FC EKF (fused range+flow+gyro-aware attitude) when it agrees with the flow, else the PC flow filter; "pc" = always the PC flow filter
    fc_xy_max_age_s: float = 0.6         # FC estimate older than this is not used (the PC flow filter takes over)
    fc_xy_min_energy: float = 0.02       # flow energy (m/s)^2 needed before the FC/flow agreement is judged
    fc_xy_agree_ratio: float = 0.6       # the best frame candidate must have this fraction of the other's error (or less), and error below the flow energy
    range_untrusted: tuple = (0.0, 0.0)  # (low, high) metres: readings in between are down-weighted in the height estimate (0,0 = trust everything)
    climb_overspeed_gain: float = 0.0    # take-off: throttle removed per m/s of climb speed above the planned climb speed + climb_overspeed_dead
    climb_overspeed_dead: float = 0.08
    ge_bonus: float = 0.0                # ground effect in the height model: extra thrust fraction on the floor (0 = none); fades out linearly to 0 at ge_h_m
    ge_h_m: float = 0.1
    range_over_read_m: float = 0.0       # >0: in the untrusted band the estimate is clamped to [reading - this, reading + 0.03] (measured over-read of the real sensor: up to 0.09 m)
    range_untrusted_r_scale: float = 40.0   # measurement variance multiplier there (x40 = sigma 6x larger)
    climb_min_jerk: bool = True          # takeoff climb follows a minimum-jerk height trajectory with a velocity feed-forward (Nimbus plans smooth trajectories too)
    ground_max_m: float = 0.30           # preflight: the range sensor must read less than this on the ground (floor 0.04-0.08, drone raised on boxes 0.17-0.21)
    range_ground_ref: float = 0.07       # the floor reading every absolute-range threshold (range_untrusted, range_cal) was measured at: readings are shifted by (this - learned ground reading), so a drone resting at 0.19 m behaves exactly like one resting at 0.07 m
    takeoff_timeout_s: float = 30.0      # TOTAL seconds without a real lift-off after the takeoff started -> stop (disarm). The real drone must not sit on the ground at high throttle
    takeoff_cap_timeout_s: float = 3.0   # ... or this long after the ground ramp reached thr_max without a lift-off (whichever comes first)
    takeoff_ramp_slow_max: float = 0.016 # creep rate the ground ramp accelerates to while NO motion is detected (smooth, over takeoff_accel_s); it slows to takeoff_ramp_slow again as soon as the estimate moves
    takeoff_accel_s: float = 6.0         # seconds without motion in the creep zone until the creep rate reaches takeoff_ramp_slow_max
    vel_sp_factor: float = 1.1           # climb-rate setpoint limit (x ref_v_max) during the take-off climb
    motion_raw_m: float = 0.03          # two raw samples in a row above this: the drone is leaving the floor, hold the ramp
    lift_raw_m: float = 0.035            # two raw range samples in a row above this height confirm the lift-off at once (the filtered estimate is ~0.3 s late)
    lift_latency_s: float = 1.0          # the lift-off is confirmed this long after it happened: the throttle ramped on by (this x ramp rate) in the meantime and the integrator starts that much lower
    ground_az_max: float = 0.0           # m/s^2: cap on the modelled acceleration before lift-off
    vel_kp: float = 0.1                 # climb velocity loop: P gain (throttle per m/s). Low on purpose: with the ~0.35 s range-estimate delay a higher gain oscillates (phase margin)
    vel_p_up: float = 0.1              # the P term may add at most this much throttle (the range estimate lags the true climb by ~0.3 s: a large positive error is mostly lag, not a real shortfall)
    vel_p_down: float = 0.05
    vel_ki: float = 0.04                 # climb velocity loop: integrator gain (throttle per (m/s error * s)); the integrator starts at the lift-off throttle and IS the hover throttle estimate
    vel_band_up: float = 0.12            # integrator bounds: [min(hover, lift) - band_down, max(hover, lift) + band_up] (hover = the expected/learned hover throttle at this voltage)
    vel_band_down: float = 0.15
    vel_cap_factor: float = 1.5          # climb speed hard cap = this x ref_v_max: above it the throttle is cut in proportion to the excess
    vel_cap_gain: float = 0.8            # throttle cut per m/s above the cap
    thr_step_max: float = 0.03           # hard limit on any throttle change per thr_step_dt (PX4-style output rate limit)
    thr_step_dt: float = 0.02
    lift_hover_tol: float = 0.12         # a lift-off throttle more than this fraction away from the expected hover throttle is not believed (false lift detection)
    soft_start_per_s: float = 0.6        # throttle may rise from zero at most this fast at the start of the takeoff (no step)
    terrain_gate: bool = False           # Nimbus-style: gate every range reading against the physics prediction; absorb sustained jumps into a terrain state
    terrain_q: float = 0.0004            # terrain random-walk variance (m^2/s)
    gate_sigma: float = 3.0
    gate_floor_m: float = 0.06           # innovation always allowed below this (sensor noise + model error)
    terrain_max_m: float = 0.40          # the terrain offset may never exceed this (a bigger 'floor change' is a sensor fault, not terrain)
    absorb_min_gap_s: float = 3.0        # at most one absorption per this many seconds
    gate_max_m: float = 0.12             # the gate never widens beyond this, however uncertain the (IMU-less) prediction becomes
    absorb_n: int = 3                    # this many consecutive rejected readings that agree with each other = the floor really changed
    absorb_spread_m: float = 0.06
    absorb_guard_s: float = 10.0         # no terrain absorption this soon after hover entry (the entry transient is ours, not the floor's) ...
    absorb_max_vz: float = 0.15          # ... nor while the estimate says we are moving vertically faster than this
    gate_reject_r_scale: float = 4.0     # rejected readings that are not a clean floor step still update the estimate with this variance multiplier (the sensor is the truth, the model only a prior)
    dropout_jump_m: float = 0.08         # a reading this far below the PREVIOUS reading (per 0.1 s) is a drop-out candidate; a real descent changes smoothly
    flight_mode: str = "pos_hold"        # 'pos_hold' (height + XY hold), 'alt_hold' (height only, XY loops off), 'stabilize' (level only, throttle frozen); applies once the take-off reaches HOVER
    mode_thr_slew_per_s: float = 0.25    # throttle change limit for mode_slew_s after a mode switch (0.25/s = 0.025 per 100 ms)
    mode_slew_s: float = 2.0
    mode_tilt_slew_deg_s: float = 8.0    # tilt command change limit when XY hold is switched off (8 deg/s = 0.8 deg per 100 ms)
    thr_slew_per_s: float = 0.0          # >0: airborne throttle command changes at most this fast (the flight profile: 0.5/s = 0.05 per 100 ms, no throttle steps that kick the drone into / out of ground effect)
    lift_ref_ge: bool = True             # with ge_bonus > 0: the hover reference = lift-off throttle corrected by the ground-effect model at the lift-off height (instead of takeoff_lift_margin)
    kv_z_climb: float = 0.0              # >0: velocity gain used for the take-off climb instead of kv_z (symmetric: brakes AND pushes, unlike climb_overspeed_gain)
    overshoot_soft: float = 0.0          # >0: the overshoot damper's cut is proportional to the excess over the threshold, reaching the full overshoot_cut at (1 + this) x the threshold (0 = the old hard step)
    use_imu: bool = False                # use the FC's window-mean vertical acceleration instead of the throttle model for the height prediction
    imu_stale_s: float = 0.35
    range_cal: tuple = ()                # ((reading_m, true_m), ...) sorted by reading: corrects the range sensor (reads too high at 20-35 cm); empty = trust the reading
    takeoff_ramp_rate: float = 0.05     # throttle added per second on the ground until the drone lifts (learns the real hover throttle)
    takeoff_lift_margin: float = 0.0     # SIGNED, only used when ge_bonus is 0 and lift_ref_ge is off; the climb velocity loop (vel_ki) finds the hover throttle, so the default has no effect and floor/boxes share one code path
    dropout_low_m: float = 0.03          # a range sample below this while the estimate says we are well above it is a drop-out
    dropout_step_m: float = 0.04         # ... or one this far below the median of the last accepted samples
    dropout_persist_s: float = 0.5       # drop-outs lasting longer than this are real (descent)
    lift_debounce_s: float = 0.3         # low reading must persist this long before lifted -> not lifted
    lift_blend_s: float = 0.0            # seconds to hand the throttle over from the creep value to the hover reference after lift-off (0 = step)
    use_lift_ref: bool = True            # hover throttle reference = what physically lifted the drone this flight (not the configured number)
    takeoff_slow_frac: float = 0.96     # from this fraction of the hover throttle the ground ramp creeps (the drone should leave the ground gently, the lift-off is noticed ~0.4 s late)
    climb_peak_frac: float = 0.45         # height change UP in flight: peak speed as a fraction of vz_lim
    descent_peak_frac: float = 0.45       # height change DOWN in flight: peak speed as a fraction of vz_lim (climbs use 0.8)
    takeoff_cap_above: float = 0.07      # the ground creep never goes above the expected hover throttle + this (measured: floor lift-off 0.59-0.63, true hover 0.62-0.65; creeping on to 0.79 made the lift-off a 2 m/s burst)
    takeoff_cap_relax_s: float = 10.0    # after this many seconds on the ground the cap rises by takeoff_cap_relax_rate per second (boxes / no ground effect need ~0.75)
    takeoff_cap_relax_rate: float = 0.015
    lift_start_above: float = 0.03       # after lift-off the velocity loop starts at most this far above the configured hover throttle, however high the lift-off throttle was (a lift that needed 0.75 on a 0.64 hover turned into a 1 m/s burst); the loop raises it again if the drone does not climb
    lift_start_frac: float = 0.7         # ... plus this fraction of the remaining gap to the lift-off throttle (boxes really hover near it)
    hover_ref_above: float = 0.12        # the lift-off value may raise the hover reference at most this far above the configured hover throttle (the velocity loop's integrator finds the rest)
    hover_learn_span: float = 0.10       # a learned hover throttle outside configured +- this is not adopted (a crawl is not a hover)
    takeoff_ramp_slow: float = 0.012    # throttle per second in the creep zone
    takeoff_start_frac: float = 0.85    # ground ramp starts at this fraction of the estimated hover throttle


class _KF2:
    """[position, velocity] filter with an acceleration input; measures either position or velocity."""

    def __init__(self, q_acc: float, r: float, p0: float = 1.0):
        self.x = [0.0, 0.0]
        self.P = [[p0, 0.0], [0.0, p0]]
        self.q, self.r = q_acc, r

    def predict(self, dt: float, a: float) -> None:
        x, v = self.x
        self.x = [x + v * dt + 0.5 * a * dt * dt, v + a * dt]
        P = self.P
        p00 = P[0][0] + dt * (P[1][0] + P[0][1]) + dt * dt * P[1][1]
        p01 = P[0][1] + dt * P[1][1]
        p10 = P[1][0] + dt * P[1][1]
        p11 = P[1][1]
        q = self.q
        self.P = [[p00 + q * dt ** 4 / 4, p01 + q * dt ** 3 / 2], [p10 + q * dt ** 3 / 2, p11 + q * dt * dt]]

    def update(self, meas: float, which: int) -> None:
        P = self.P
        s = P[which][which] + self.r
        k0, k1 = P[0][which] / s, P[1][which] / s
        y = meas - self.x[which]
        self.x = [self.x[0] + k0 * y, self.x[1] + k1 * y]
        p = [[P[0][0] - k0 * P[which][0], P[0][1] - k0 * P[which][1]], [P[1][0] - k1 * P[which][0], P[1][1] - k1 * P[which][1]]]
        self.P = [[p[0][0], p[0][1]], [p[1][0], p[1][1]]]


class _KF3:
    """Vertical filter [z, vz, b]: b is an acceleration bias (model error: weight, battery, thrust curve) learned from the range sensor."""

    def __init__(self, q_acc: float = 1.0, q_bias: float = 0.6, r: float = 0.02 ** 2):
        self.x = [0.0, 0.0, 0.0]
        self.P = [[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]]
        self.qa, self.qb, self.r = q_acc, q_bias, r

    def predict(self, dt: float, a_model: float) -> None:
        z, v, b = self.x
        a = a_model + b
        self.x = [z + v * dt + 0.5 * a * dt * dt, v + a * dt, b]
        F = [[1, dt, 0.5 * dt * dt], [0, 1, dt], [0, 0, 1]]
        P = self.P
        FP = [[sum(F[i][k] * P[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
        Pn = [[sum(FP[i][k] * F[j][k] for k in range(3)) for j in range(3)] for i in range(3)]
        g = [0.5 * dt * dt, dt, 0.0]
        for i in range(3):
            for j in range(3):
                Pn[i][j] += self.qa * g[i] * g[j]
        Pn[2][2] += self.qb * dt
        self.P = Pn

    def update(self, z_meas: float, r_scale: float = 1.0) -> None:
        P = self.P
        s = P[0][0] + self.r * r_scale
        K = [P[i][0] / s for i in range(3)]
        y = z_meas - self.x[0]
        self.x = [self.x[i] + K[i] * y for i in range(3)]
        self.P = [[P[i][j] - K[i] * P[0][j] for j in range(3)] for i in range(3)]


class _KF4(_KF3):
    """Vertical filter [z, vz, b, t] (Nimbus keeps a TERRAIN state too): z = height above the take-off floor (global), t = how much the floor under the drone is above that
    floor, b = acceleration bias. The range sensor measures z - t. With t frozen (P_tt = 0) this is exactly _KF3."""

    def __init__(self, q_acc: float = 1.0, q_bias: float = 0.6, r: float = 0.02 ** 2, q_terrain: float = 0.0):
        self.x = [0.0, 0.0, 0.0, 0.0]
        self.P = [[1.0, 0, 0, 0], [0, 1.0, 0, 0], [0, 0, 1.0, 0], [0, 0, 0, 0.0]]
        self.qa, self.qb, self.r, self.qt = q_acc, q_bias, r, q_terrain

    def predict(self, dt: float, a_model: float) -> None:
        z, v, b, t = self.x
        a = a_model + b
        self.x = [z + v * dt + 0.5 * a * dt * dt, v + a * dt, b, t]
        F = [[1, dt, 0.5 * dt * dt, 0], [0, 1, dt, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
        P = self.P
        FP = [[sum(F[i][k] * P[k][j] for k in range(4)) for j in range(4)] for i in range(4)]
        Pn = [[sum(FP[i][k] * F[j][k] for k in range(4)) for j in range(4)] for i in range(4)]
        g = [0.5 * dt * dt, dt, 0.0, 0.0]
        for i in range(4):
            for j in range(4):
                Pn[i][j] += self.qa * g[i] * g[j]
        Pn[2][2] += self.qb * dt
        Pn[3][3] += self.qt * dt
        self.P = Pn

    H = (1.0, 0.0, 0.0, -1.0)

    def innovation(self, meas: float, r_scale: float = 1.0) -> tuple[float, float]:
        """(y, S): measurement minus predicted range, and its variance."""
        P, H = self.P, self.H
        S = sum(H[i] * P[i][j] * H[j] for i in range(4) for j in range(4)) + self.r * r_scale
        return meas - (self.x[0] - self.x[3]), S

    def update(self, z_meas: float, r_scale: float = 1.0) -> None:
        P, H = self.P, self.H
        y, S = self.innovation(z_meas, r_scale)
        PH = [sum(P[i][j] * H[j] for j in range(4)) for i in range(4)]
        K = [PH[i] / S for i in range(4)]
        self.x = [self.x[i] + K[i] * y for i in range(4)]
        self.P = [[P[i][j] - K[i] * PH[j] for j in range(4)] for i in range(4)]

    def absorb_terrain(self, delta: float, var: float = 0.01) -> None:
        """The floor moved by delta under us (range jumped, physics says we did not): move the terrain state, not the height."""
        self.x[3] -= delta
        self.P[3][3] = max(self.P[3][3], var)
        for k in range(4):
            if k != 3:
                self.P[3][k] = self.P[k][3] = 0.0


class HoverController:
    def __init__(self, cfg: Optional[HoverConfig] = None):
        self.cfg = cfg or HoverConfig()
        c = self.cfg
        self.state = HoverState.IDLE
        self.kz = _KF4()
        self.kx = _KF2(q_acc=1.0, r=0.06 ** 2)
        self.ky = _KF2(q_acc=1.0, r=0.06 ** 2)
        self.z_ref = 0.0
        self.x_ref = self.y_ref = 0.0
        self.integ = 0.0
        self.thr_eff_est = 0.0
        self.tilt_x_est = self.tilt_y_est = 0.0     # estimated tilt (rad) from our own commands through the attitude lag
        self.batt_v = c.batt_v0
        self._batt_seen = False
        self.v_start = None                          # pack reading at the start of this flight: the throttle scales with the voltage RELATIVE to it (a constant scale error of the reading cancels)
        self.pitch_deg = self.roll_deg = 0.0
        self.last_optrange = -1e9
        self.t_state = 0.0
        self.t_last = None
        self.xy_disabled = False                    # set after a position runaway: stay level and land vertically
        self._init_fc_xy()
        self.lifted, self.ground_thr = False, None
        self.dropouts_rejected = 0
        self._rng_hist = []; self._drop_t0 = None; self._low_t0 = None; self._last_raw = (-1e9, 0.0)
        self.lift_thr = None; self.last_range_m = None; self.land_i = 0.0
        self._zm_prev = 0.0; self._zm_mot = 0; self._zm_hi = 0; self._slow_t = 0.0; self._ramp_rate = 0.0; self.az_ff = 0.0; self.ground_raw = 0.0; self._ground_buf = []; self.thr_i = None; self._nomo = 0.0; self._t_cap = None; self._aborted = False; self._stuck_t = 0.0
        self.imu_az = 0.0; self.t_imu = -1e9
        self.terrain_events = 0
        self._t_absorb = -1e9
        self._sus = []
        self.ground = 0.0                            # what the range sensor reads on the floor (measured before takeoff)
        self._raw_hist: deque = deque(maxlen=64)     # (time, raw range) while idle: the ground reference comes from here
        self._lift_cnt = 0
        self.t_lift = None; self.thr_at_lift = 0.0
        self._climb = None
        self._traj = None; self._traj_tag = None      # jerk-limited vertical reference (JerkLimited1D) and what it is for: ("up", cap) | ("hold", h) | ("land",)
        self._land_bias = None
        self._ramp = None                             # landing: (t0, throttle at t0) of the throttle ramp to idle
        self._contact_s = 0.0
        self._dt = 0.01
        self._learn: deque = deque(maxlen=3000); self._learn_dt = 0.01; self._learn_adopted = None
        self.ix = [0.0, 0.0]                          # XY position integrals (m*s)
        self.vz_ff = 0.0
        self.hover_ref = None                        # (throttle, pack V) that physically held the drone at lift-off: replaces the configured hover throttle for this flight
        self._flags: set = set()
        self._thr_hist = deque()
        self._cap_t = 0.0
        self.thr_learned = None                    # smoothed throttle that actually holds the drone in hover (used for the blind descent)
        self.runaway_since = None
        self._now = 0.0
        self.opt_times: deque = deque(maxlen=128)
        self.refused: list = []
        self.sign_done = False
        self.sign_base = [0.0, 0.0]
        self.sign_peak = [0.0, 0.0]
        self.flow_raw = [(0.0, 0.0), (0.0, 0.0)]        # last two raw flow readings (vx, vy), straight from the sensor
        self._base_samples = [[], []]
        self.cmd = Controls()
        self.reason = ""
        self.events: list = []
        self._xy_gate, self._xy_gate_reason = False, "no flow data"
        self._flow_q, self._flow_t, self._xy_fast_until = 0, -1e9, -1e9
        self._xy_reset()
        self.flight_mode = self.cfg.flight_mode
        self._mode_applied, self._stab_thr, self._mode_slew_until = "pos_hold", 0.0, -1e9

    MODES = ("stabilize", "alt_hold", "pos_hold")

    def set_flight_mode(self, mode: str) -> bool:
        """Choose the flight mode (any time, also before take-off). It takes effect when the controller is in HOVER; before that the take-off runs as always."""
        if mode not in self.MODES:
            return False
        self.flight_mode = mode
        return True

    def _mode_target(self) -> str:
        if self.state == HoverState.HOVER:
            return self.flight_mode
        if self.state in (HoverState.LAND, HoverState.SENSOR_LOSS, HoverState.DONE):
            return self._mode_applied                       # a landing / failsafe keeps what was flying
        return "pos_hold"

    def _mode_transition(self, old: str, new: str) -> None:
        c = self.cfg
        self._mode_applied = new
        self.events.append((self._now, f"flight mode active: {new}"))
        self._mode_slew_until = self._now + c.mode_slew_s
        if new != "pos_hold":                              # XY loops off: gates closed, integrators and learned lean cleared
            self._xy_gate, self._damp_gate, self._w = False, False, 0.0
            self.ix = [0.0, 0.0]; self._lean = [0.0, 0.0]
            self.x_ref, self.y_ref = self.kx.x[0], self.ky.x[0]
        else:                                              # the gate reopens when valid and holds where we are now
            self.origin = None; self._xy_gate, self._damp_gate, self._w = False, False, 0.0
            self.x_ref, self.y_ref = self.kx.x[0], self.ky.x[0]
        if new == "stabilize":
            recent = sorted(t for t, _ in list(self._learn)[-300:])                    # the learned hover throttle when there is settled hover behind us, else the hover base (the command right now may sit in a transient, e.g. the overshoot damper's cut)
            self._stab_thr = recent[len(recent) // 2] if len(recent) * self._learn_dt >= 1.0 else self._thr_hover_now()
            self._traj = self._traj_tag = None; self.vz_ff = 0.0
        elif old == "stabilize":                           # height loop back on: reference = where we are, integrator carries the frozen throttle (no step)
            self.z_ref = self.kz.x[0]; self.vz_ff = 0.0; self._traj = self._traj_tag = None
            self.integ = min(max(self._stab_thr - self._thr_hover_now(), -c.i_max), c.i_max)

    def _xy_reset(self) -> None:
        self.origin = None                            # horizontal estimate at lift-off: the hold reference when the full hold opens
        self._fbuf, self._cbuf = [], []               # last flow / FC velocity samples (median input)
        self._qbuf = []                               # last flow qualities
        self._flow_hi = 0                             # consecutive raw flow samples above xy_max_speed_ms
        self._flow_med = None                         # (time, vx, vy) median of the PC flow
        self._damp_gate = False; self._w = 0.0; self._out = [0.0, 0.0]; self._lean = [0.0, 0.0]

    @property
    def xy_gate_open(self) -> bool: return self._xy_gate

    @property
    def xy_gate_reason(self) -> str: return self._xy_gate_reason

    # --- public ----------------------------------------------------------------------------
    @property
    def z(self) -> float: return self.kz.x[0]

    def _set_state(self, now: float, st: HoverState, why: str = "") -> None:
        if st != self.state:
            self.events.append((now, f"{self.state.value} -> {st.value}" + (f" ({why})" if why else "")))
            self.state, self.t_state, self.reason = st, now, why
            if st == HoverState.LAND:
                self._ramp, self._contact_s, self._traj, self._traj_tag, self._land_bias = None, 0.0, None, None, None
                self.land_i = 0.0
                self.ix = [0.0, 0.0]

    def observe(self, now: float, samples) -> None:
        """Feed telemetry while idle (so preflight can judge the data rate) without producing any control output."""
        for smp in samples:
            self._ingest(now, smp)

    def telemetry_health(self, now: float, window: float = 2.0) -> tuple[float, float]:
        """(range samples per second, longest gap in seconds) over the last `window` seconds."""
        t = [x for x in self.opt_times if now - x <= window]
        if not t:
            return 0.0, window
        pts = [now - window] + t + [now]
        return len(t) / window, max(b - a for a, b in zip(pts, pts[1:]))

    def preflight(self, now: float) -> tuple[bool, list]:
        c = self.cfg
        rate, gap = self.telemetry_health(now)
        why = []
        if rate < c.min_rate_hz:
            why.append(f"range data too slow: {rate:.1f} Hz (need {c.min_rate_hz:.0f}); raise the ELRS telemetry ratio")
        if gap > c.max_gap_s:
            why.append(f"range data has a {gap:.2f} s gap (limit {c.max_gap_s} s)")
        return (not why), why                     # the battery never blocks a takeoff: it only warns (see battery_warning())

    def battery_warning(self) -> str:
        """Text of the battery warning, or '' (warning only: the pilot decides)."""
        c = self.cfg
        if self.batt_v < c.batt_warn_v:
            return f"WARNING battery {self.batt_v:.1f} V is below {c.batt_warn_v:.1f} V (warning only: nothing is blocked, you decide)"
        return ""

    def start(self, now: float, force: bool = False) -> bool:
        c = self.cfg
        w = self.battery_warning()
        if w:
            self.events.append((now, w))
        ok, why = self.preflight(now)
        if not ok and not force:
            self.refused = why
            self.events.append((now, "takeoff refused: " + "; ".join(why)))
            return False
        self.refused = []
        recent = sorted(r for t, r in self._raw_hist if now - t <= 1.5)
        self._rng_hist = []; self._drop_t0 = None; self._last_raw = (-1e9, 0.0)
        if len(recent) >= 5:
            self.ground_raw = recent[len(recent) // 2]
            self.ground = self._cal_range(self.ground_raw)                # median of the last 1.5 s: the sensor's reading on the floor (same calibration as the flight readings)
        elif not force:
            self.refused = ["no recent range readings to measure the ground reference"]
            self.events.append((now, "takeoff refused: " + self.refused[0])); return False
        else:
            self.ground = 0.0; self.ground_raw = c.range_ground_ref
        self._ground_buf = []; self.thr_i = None; self._nomo = 0.0; self._t_cap = None; self._aborted = False; self._stuck_t = 0.0
        if self.ground > c.ground_max_m and not force:
            self.refused = [f"range sensor reads {self.ground:.2f} m on the ground (limit {c.ground_max_m:.2f}): something under the drone or the sensor is blocked"]
            self.events.append((now, "takeoff refused: " + self.refused[0])); return False
        self.events.append((now, f"ground reference {self.ground:.3f} m (heights are relative to it)"))
        self._lift_cnt = 0; self._zm_prev = 0.0; self._zm_mot = 0; self._zm_hi = 0
        self.kz = _KF4(q_terrain=c.terrain_q if c.terrain_gate else 0.0); self.kx = _KF2(1.0, 0.06 ** 2); self.ky = _KF2(1.0, 0.06 ** 2)
        self._sus = []                               # consecutive range readings the physics model rejected (signed innovations)
        self.lifted, self.ground_thr = False, None
        self.lift_thr = None; self.hover_ref = None; self._flags = set(); self._climb = None; self.vz_ff = 0.0; self.t_lift = None
        self._traj = self._traj_tag = self._ramp = None; self._contact_s = 0.0; self.ix = [0.0, 0.0]
        self._learn.clear(); self._learn_adopted = None; self._thr_hist.clear(); self._cap_t = 0.0
        self.v_start = self.batt_v
        self.z_ref, self.integ, self.t_last = 0.0, 0.0, now
        self.xy_disabled = False
        self._xy_gate, self._xy_gate_reason = False, "take-off"
        self._flow_q, self._flow_t, self._xy_fast_until = 0, -1e9, -1e9
        self._xy_reset()
        self.thr_learned, self.runaway_since = None, None
        self.sign_peak, self.sign_base, self._base_samples = [0.0, 0.0], [0.0, 0.0], [[], []]
        self.sign_done = False
        self.last_optrange = now
        self._set_state(now, HoverState.TAKEOFF)
        return True

    def land(self, now: float, why: str = "operator") -> None:
        if why not in ("operator", "scheduled", "user", "landing"):
            self._aborted = True                                        # not a normal landing: nothing is learned from this flight
        if self.state in (HoverState.TAKEOFF, HoverState.SIGN_CHECK, HoverState.HOVER):
            if abs(self.kz.x[3]) > 1e-6:                                # land onto whatever is under us now: height above THAT surface, terrain back to zero
                self.kz.x[0] -= self.kz.x[3]; self.kz.x[3] = 0.0; self.z_ref = min(self.z_ref, self.kz.x[0] + 0.05)
            self._set_state(now, HoverState.LAND, why)

    def update(self, now: float, samples, dt: float) -> Controls:
        c = self.cfg
        self._now = now
        self._dt = dt
        if self.state in (HoverState.IDLE, HoverState.DONE):
            self.cmd = Controls(arm=False)
            return self.cmd
        self._predict(dt)
        for s in samples:
            self._ingest(now, s)
        self._supervise(now)
        if self.state == HoverState.DONE:
            self.cmd = Controls(arm=False)
            return self.cmd
        tgt = self._mode_target()
        if tgt != self._mode_applied:
            self._mode_transition(self._mode_applied, tgt)
        thr = self._altitude(now, dt)
        pitch, roll = self._horizontal(dt)
        ramping = self.state == HoverState.LAND and self._ramp is not None
        thr = min(max(thr, 0.0 if ramping else (c.thr_min_air if self.z > 0.03 else 0.0)), c.thr_max)
        if self.lifted and self.state in (HoverState.TAKEOFF, HoverState.SIGN_CHECK, HoverState.HOVER) and c.thr_slew_per_s > 0:
            lim = c.thr_slew_per_s * dt
            thr = min(max(thr, self.cmd.throttle - lim), self.cmd.throttle + lim)      # no throttle steps in the air (the overshoot damper's cut becomes a ramp)
        if self.lifted and self.state == HoverState.HOVER and now < self._mode_slew_until:
            lim = c.mode_thr_slew_per_s * dt                                            # just after a mode switch: the throttle glides (no step), whatever the normal slew setting
            thr = min(max(thr, self.cmd.throttle - lim), self.cmd.throttle + lim)
        if self.cmd.arm and c.thr_step_max > 0 and dt > 0:           # output rate limit: no throttle step larger than thr_step_max per thr_step_dt
            lim = c.thr_step_max * dt / c.thr_step_dt
            thr = min(max(thr, self.cmd.throttle - lim), self.cmd.throttle + lim)
        if c.cap_land_s > 0 and self.lifted and self.state in (HoverState.TAKEOFF, HoverState.HOVER):
            if thr >= c.thr_max - 0.005 and self.kz.x[1] < 0.10:
                self._cap_t += dt
                if self._cap_t >= c.cap_land_s:
                    self.events.append((now, f"throttle at the cap {c.thr_max:.2f} for {c.cap_land_s:.1f} s and not climbing: the thrust limit is reached, landing gently"))
                    self._cap_t = 0.0
                    self.land(now, f"thrust limit: throttle at the cap {c.thr_max:.2f} and not climbing")
            else:
                self._cap_t = max(self._cap_t - dt, 0.0)
        if c.thr_delay_s > 0:
            h = self._thr_hist; h.append((now, thr))
            while len(h) > 1 and h[1][0] <= now - c.thr_delay_s:
                h.popleft()
            thr_m = h[0][1]
        else:
            thr_m = thr
        self.thr_eff_est += (thr_m - self.thr_eff_est) * dt / c.tau_motor
        lim = math.radians(c.angle_limit_deg)
        self.tilt_x_est += (pitch * lim - self.tilt_x_est) * dt / c.tau_att          # physical tilt we asked for
        self.tilt_y_est += (roll * lim - self.tilt_y_est) * dt / c.tau_att
        self.cmd = Controls(roll=roll * c.roll_sign, pitch=pitch * c.pitch_sign, yaw=0.0, throttle=thr, arm=True)   # hardware signs applied once, here
        return self.cmd

    # --- estimation ------------------------------------------------------------------------
    def _cal_range(self, z: float) -> float:
        pts = self.cfg.range_cal
        if not pts or z <= pts[0][0]:
            return z
        for (r0, t0), (r1, t1) in zip(pts, pts[1:]):
            if z <= r1:
                return t0 + (t1 - t0) * (z - r0) / (r1 - r0)
        return pts[-1][1] + (z - pts[-1][0])

    def _ge(self, z: float) -> float:
        """Ground-effect thrust factor at height z above the take-off point. The effect belongs to the distance from the REAL floor: a drone resting 0.12 m higher (boxes: ground reading 0.19 instead of 0.07) is
        already out of it (measured: leaves a box at the hover throttle, the floor 0.05 below it), so the learned rest height above the floor reading shifts the curve."""
        c = self.cfg
        if not c.ge_bonus:
            return 1.0
        off = max(self.ground_raw - c.range_ground_ref - 0.03, 0.0)
        return 1.0 + c.ge_bonus * max(0.0, 1.0 - (max(z, 0.0) + off) / c.ge_h_m)

    def _shift(self) -> float:
        """Range offset that maps the learned ground reading onto range_ground_ref (only DOWNWARDS: a drone raised on boxes; a ground reading below the reference needs no shift)."""
        return min(self.cfg.range_ground_ref - self.ground_raw, 0.0)

    def _zrel(self, raw: float) -> float:
        """Height above the learned ground reading from a raw range reading. The range calibration and the untrusted band are curves of the distance to the floor
        as measured on the floor (reading 0.07 m at rest): the reading is shifted so that the learned ground reading maps to that, whatever it really is."""
        c = self.cfg
        shift = self._shift()
        return max(self._cal_range(raw + shift) - self._cal_range(self.ground_raw + shift), 0.0)

    def _hover0(self) -> tuple[float, float]:
        """(hover throttle, pack voltage it applies at): from the flight when known (lift-off, later the settled-hover median), else the configured hover throttle taken at the pack reading at the start of the flight (only RELATIVE voltage changes matter)."""
        c = self.cfg
        return self.hover_ref if self.hover_ref is not None else (c.thr_hover0, self.v_start if self.v_start is not None else self.batt_v)

    def _thrust_ratio(self) -> float:
        c = self.cfg
        h0, v0 = self._hover0()
        u = max(self.thr_eff_est - c.thr_idle, 0.0) / max(h0 - c.thr_idle, 1e-6)
        ge = self._ge(self.kz.x[0])                  # ground effect: extra thrust near the floor (measured on the real drone)
        return (u ** c.thrust_exp) * (self.batt_v / v0) ** 2 * ge

    def _lead(self, now: float) -> tuple:
        """(dz, dvz): where the drone will be thr_delay_s from now compared with the estimate, from the commands already sent but not yet effective (the estimate is built
        from the delayed command, so it lags by that delay; the loop must not wait for the plant it already commanded)."""
        c = self.cfg
        d = c.thr_delay_s
        h0, v0 = self._hover0()
        hist = self._thr_hist
        if d <= 0 or len(hist) < 2:
            return 0.0, 0.0
        n = 8; dt = d / n; dz = dvz = 0.0
        vz = self.kz.x[1]
        for k in range(n):
            tt = now - d + (k + 0.5) * dt                       # the command that takes effect at now + (k+0.5) dt
            thr = hist[0][1]
            for (ts, th) in hist:
                if ts > tt: break
                thr = th
            u = max(thr - c.thr_idle, 0.0) / max(h0 - c.thr_idle, 1e-6)
            az = G * ((u ** c.thrust_exp) * (self.batt_v / v0) ** 2 - 1.0)
            az -= 0.15 * (vz + dvz)
            dz += (vz + dvz) * dt + 0.5 * az * dt * dt
            dvz += az * dt
        # the part of the history older than the delay is already in the estimate (the model integrates it): only the in-flight part is added
        return dz, dvz

    def _predict(self, dt: float) -> None:
        c = self.cfg
        az = G * (self._thrust_ratio() * math.cos(self.tilt_x_est) * math.cos(self.tilt_y_est) - 1.0) - 0.15 * self.kz.x[1]
        if c.use_imu and self._now - self.t_imu < c.imu_stale_s and self.lifted:
            az = self.imu_az                         # measured acceleration (already includes drag and the real thrust); the bias state absorbs its constant error
        if not self.lifted:
            az = min(az, c.ground_az_max)                  # still on the ground: the throttle model must not invent a climb (a ramp far above the expected hover throttle: boxes, another battery); only the range sensor shows the lift-off
        if self.kz.x[0] <= 0.0 and az < 0:
            az = 0.0
            self.kz.x[1] = max(self.kz.x[1], 0.0)
        self.kz.predict(dt, az)
        if self.kz.x[0] < 0.0:
            self.kz.x[0] = 0.0
            self.kz.x[1] = max(self.kz.x[1], 0.0)
        ax = G * math.tan(self.tilt_x_est) - c.drag * self.kx.x[1]
        ay = G * math.tan(self.tilt_y_est) - c.drag * self.ky.x[1]
        if self.kz.x[0] < 0.03:                       # on the ground: no horizontal motion
            ax = ay = 0.0
            self.kx.x[1] *= 0.5; self.ky.x[1] *= 0.5
            self.kx.P = [[1e-6, 0.0], [0.0, 0.06 ** 2]]; self.ky.P = [[1e-6, 0.0], [0.0, 0.06 ** 2]]      # on the floor the position and velocity are known (zero): no stale covariance that would make the first flow samples jump the position
        self.kx.predict(dt, ax)
        self.ky.predict(dt, ay)

    def _ingest(self, now: float, s) -> None:
        c = self.cfg
        d = s.data
        if s.kind == "optrange":
            self.last_optrange = now
            self.last_range_m = d["distance_m"]
            if d["distance_m"] <= 0.005 and self.ground > 0.02:    # the decoder reports 0.0 for an invalid/out-of-range reading (seen when the drone tilts): not a measurement
                return
            self.opt_times.append(now)
            tilt_cos = max(math.cos(math.radians(self.pitch_deg)) * math.cos(math.radians(self.roll_deg)), 0.5)
            if self.state in (HoverState.IDLE, HoverState.DONE):
                self._raw_hist.append((now, d["distance_m"]))
            z_meas = self._zrel(d["distance_m"] * tilt_cos)                                    # height above the floor reading measured before takeoff
            if self.state == HoverState.TAKEOFF and not self.lifted and self._ground_buf is not None:
                if now - self.t_state <= 1.0:
                    self._ground_buf.append(d["distance_m"])             # armed on the ground: refine the ground reading from the first second of samples
                else:
                    gb = sorted(self._ground_buf); self._ground_buf = None
                    if len(gb) >= 5 and gb[-1] - gb[0] <= 0.03:
                        self.ground_raw = gb[len(gb) // 2]; self.ground = self._cal_range(self.ground_raw)
                        z_meas = self._zrel(d["distance_m"] * tilt_cos)
            self._zm_mot = self._zm_mot + 1 if (z_meas > c.motion_raw_m and self._zm_prev > c.motion_raw_m) else 0         # two raw samples in a row above 3 cm: the drone is moving (the filtered estimate is late)
            self._zm_hi = self._zm_hi + 1 if (z_meas > c.lift_raw_m and self._zm_prev > c.lift_raw_m) else 0
            self._zm_prev = z_meas
            dly = c.sensor_delay_s
            vz = self.kz.x[1]
            skip = False
            if self.lifted or self.kz.x[0] > 0.04:                # drop-out rejection: single floor-return samples while clearly above it
                hist = self._rng_hist
                med = sorted(hist)[len(hist) // 2] if len(hist) >= 3 else None
                last_t, last_raw = self._last_raw
                jump = c.dropout_jump_m * max(1.0, (now - last_t) / 0.1)       # a real descent changes the reading smoothly from one sample to the next; a drop-out is a jump
                drop = (z_meas < c.dropout_low_m or (med is not None and z_meas < med - c.dropout_step_m and z_meas < last_raw - jump)) and self.kz.x[0] > z_meas + c.dropout_step_m
                self._last_raw = (now, z_meas)
                if drop:
                    if self._drop_t0 is None: self._drop_t0 = now
                    if now - self._drop_t0 <= c.dropout_persist_s:
                        self.dropouts_rejected += 1
                        skip = True                              # the height update only: the flow part of the sample is still good
                else:
                    self._drop_t0 = None
                if not skip:
                    hist.append(z_meas); del hist[:-5]
            if skip:
                pass
            elif z_meas < 0.04 and not self.lifted:              # sitting on the ground: believe the range sensor, no predicted climb
                self.kz.x[0], self.kz.x[1], self.kz.x[2] = z_meas, min(vz, 0.0), self.kz.x[2]
            else:
                lo, hi = c.range_untrusted
                d_eq = d["distance_m"] + self._shift()      # the band is in range units measured with the floor at range_ground_ref: shift to height above THIS ground
                untrusted = lo < d_eq < hi and self.lifted      # the sensor over-reads here (measured +9 cm at 20-30 cm): ride on the physics model, not on these readings
                rs = c.range_untrusted_r_scale if untrusted else 1.0
                meas = z_meas + vz * dly                               # bring the old reading forward to now
                gate_on = (c.terrain_gate and self.lifted and self.state in (HoverState.SIGN_CHECK, HoverState.HOVER) and self.kz.x[0] >= c.range_untrusted[1])
                if gate_on:
                    y, S = self.kz.innovation(meas, rs)
                    allowed = min(max(c.gate_sigma * math.sqrt(max(S, 1e-9)), c.gate_floor_m), c.gate_max_m)
                    if abs(y) <= allowed:
                        self._sus.clear(); self.kz.update(meas, rs)       # physically plausible: a normal update
                    else:                                              # the physics model cannot explain this reading
                        self._sus.append(y)
                        if len(self._sus) >= c.absorb_n:
                            mean = sum(self._sus) / len(self._sus)
                            settled = self.state == HoverState.HOVER and now - self.t_state >= c.absorb_guard_s and abs(self.kz.x[1]) < c.absorb_max_vz and "overshoot" not in self._flags
                            if settled and max(self._sus) - min(self._sus) <= c.absorb_spread_m and abs(self.kz.x[3] - mean) <= c.terrain_max_m and now - self._t_absorb >= c.absorb_min_gap_s:       # consistent, in a settled hover: the FLOOR changed (box, rug, step), not us
                                self.kz.absorb_terrain(mean); self._t_absorb = now
                                self.terrain_events += 1
                                self.events.append((now, f"floor changed by {-mean * 100:+.0f} cm under the drone: absorbed into the terrain state (height unchanged)"))
                                self._sus.clear()
                            else:
                                del self._sus[0]                       # not a clean floor step (or not settled): a SUSTAINED disagreement means the model is wrong, not the sensor
                                self.kz.update(meas, rs * c.gate_reject_r_scale)          # follow the sensor (down-weighted): a fast real descent must never leave the estimate behind
                else:
                    self.kz.update(meas, rs)
                    if untrusted and c.range_over_read_m > 0.0:          # the sensor only reads too HIGH here (by at most range_over_read_m): the true height lies in that bracket, so the physics model cannot drift out of it
                        dr = d_eq
                        ob = c.range_over_read_m * (min(max((dr - 0.06) / 0.14, 0.0), 1.0) if dr < 0.32 else min(max((0.47 - dr) / 0.15, 0.0), 1.0))      # the over-read grows from ~0 near the floor to the full value by 0.2 m and fades out by ~0.45 m
                        self.kz.x[0] = min(max(self.kz.x[0], meas - ob), meas + 0.03)
            q = d.get("quality", 255)
            self._qbuf = (self._qbuf + [q])[-3:]
            self._flow_q, self._flow_t = max(self._qbuf), now          # the gate closes on 3 low-quality samples in a row, not on one
            flow_ok = q >= c.flow_min_quality and self.kz.x[0] > (c.xy_damp_min_height_m - c.xy_damp_hyst_m if self.lifted else 0.05)
            self._flow_b = (now, d["flow_vx"], d["flow_vy"], flow_ok)
            if flow_ok:
                self.flow_raw = [self.flow_raw[1], (d["flow_vx"], d["flow_vy"])]
                if math.hypot(d["flow_vx"], d["flow_vy"]) > c.xy_max_speed_ms:
                    self._flow_hi += 1                               # a single spike is dropped; only a sustained run of fast samples is believed (and closes the gate)
                else:
                    self._flow_hi = 0
                if self._flow_hi >= c.xy_median_n:
                    self._xy_fast_until = now + c.xy_speed_hold_s
                if self._flow_hi == 0 or self._flow_hi >= c.xy_median_n:
                    self._fbuf = (self._fbuf + [(d["flow_vx"], d["flow_vy"])])[-c.xy_median_n:]
                if self._fbuf:
                    mx, my = self._median(self._fbuf)
                    self._flow_med = (now, mx, my)
                    if not self.fc_xy_in_use(now):                   # the FC estimate drives XY when it is trusted and fresh; the PC flow filter is the fallback
                        self.kx.update(mx, 1)
                        self.ky.update(my, 1)
                self.xy_active = "fc" if self.fc_xy_in_use(now) else "pc"
        elif s.kind == "fcxy":
            self._ingest_fcxy(now, d)
        elif s.kind == "imu":
            self.imu_az, self.t_imu = d["az_mean"], now
        elif s.kind == "attitude":
            self.pitch_deg, self.roll_deg = d["pitch_deg"], d["roll_deg"]
        elif s.kind == "battery":
            if not self._batt_seen:
                self.batt_v, self._batt_seen = d["voltage_v"], True
            else:
                self.batt_v += 0.3 * (d["voltage_v"] - self.batt_v)

    # --- FC horizontal estimate -------------------------------------------------------------------
    def _init_fc_xy(self) -> None:
        self.fc_xy_t = -1e9                       # time of the last valid FC velocity
        self.fc_xy_sign = 0                       # +1 / -1 = yaw convention that agrees with the flow, 0 = not yet known
        self._fcxy_err = {1: 0.0, -1: 0.0}; self._fcxy_energy = 0.0
        self._flow_b = None                       # (time, forward, right, quality-ok) of the latest PC flow
        self.xy_active = "pc"

    def fc_xy_in_use(self, now: float) -> bool:
        return self.cfg.xy_source == "fc" and self.fc_xy_sign != 0 and now - self.fc_xy_t <= self.cfg.fc_xy_max_age_s

    @staticmethod
    def _to_body(vx: float, vy: float, yaw_deg: float, sign: int) -> tuple:
        a = math.radians(yaw_deg) * sign
        return math.cos(a) * vx + math.sin(a) * vy, -math.sin(a) * vx + math.cos(a) * vy

    def _ingest_fcxy(self, now: float, d: dict) -> None:
        """FC EKF velocity (world frame of the FC's attitude quaternion) -> body forward/right. The yaw sign convention is not assumed: both candidates are scored
        against the PC's own flow (forward/right) while the drone moves, and the better one is used only when it clearly agrees; until then the PC flow filter runs."""
        c = self.cfg
        if c.xy_source != "fc":
            return
        vx, vy, yaw = d["vx_mps"], d["vy_mps"], d["yaw_deg"]
        fb = self._flow_b
        if fb is not None and now - fb[0] < 0.25 and fb[3]:
            for sgn in (1, -1):
                bx, by = self._to_body(vx, vy, yaw, sgn)
                self._fcxy_err[sgn] = 0.98 * self._fcxy_err[sgn] + (bx - fb[1]) ** 2 + (by - fb[2]) ** 2
            self._fcxy_energy = 0.98 * self._fcxy_energy + fb[1] ** 2 + fb[2] ** 2
            n = 50.0                                                  # effective window ~50 samples
            if self._fcxy_energy / n >= c.fc_xy_min_energy:
                best = 1 if self._fcxy_err[1] <= self._fcxy_err[-1] else -1
                e_b, e_o = self._fcxy_err[best], self._fcxy_err[-best]
                agree = e_b <= c.fc_xy_agree_ratio * e_o and e_b <= 0.5 * self._fcxy_energy
                if agree and best != self.fc_xy_sign:
                    self.fc_xy_sign = best
                    self.events.append((now, f"FC horizontal estimate agrees with the flow (yaw convention {'+' if best > 0 else '-'}): XY now from the FC EKF"))
                elif not agree and self.fc_xy_sign != 0 and e_b > 0.9 * self._fcxy_energy:
                    self.fc_xy_sign = 0
                    self.events.append((now, "FC horizontal estimate no longer agrees with the flow: XY back on the PC flow filter"))
        if self.fc_xy_sign == 0 or not d.get("valid", True):
            return
        bx, by = self._to_body(vx, vy, yaw, self.fc_xy_sign)
        self.fc_xy_t = now
        self._cbuf = (self._cbuf + [(bx, by)])[-c.xy_median_n:]
        bx, by = self._median(self._cbuf)
        fm = self._flow_med
        if fm is not None and now - fm[0] < 0.5 and math.hypot(bx - fm[1], by - fm[2]) > c.xy_disagree_ms:       # the two estimates disagree: the smaller one is the safer one to damp against
            if math.hypot(fm[1], fm[2]) < math.hypot(bx, by):
                bx, by = fm[1], fm[2]
        self.kx.update(bx, 1)
        self.ky.update(by, 1)

    @staticmethod
    def _median(buf) -> tuple:
        """Median of the last samples; the newest sample is used itself when it is within 0.15 m/s of that median (a median alone adds about 0.2 s of lag to every real change)."""
        n = len(buf)
        med = tuple(sorted(b[a] for b in buf)[n // 2] for a in (0, 1))
        return buf[-1] if math.hypot(buf[-1][0] - med[0], buf[-1][1] - med[1]) <= 0.15 else med

    # --- supervision (safety envelope) ---------------------------------------------------------
    def _supervise(self, now: float) -> None:
        c, st = self.cfg, self.state
        airborne = self.z > 0.10
        if max(abs(self.pitch_deg), abs(self.roll_deg)) > c.tilt_abort_deg:
            self._set_state(now, HoverState.DONE, "tilt exceeded"); return
        stale = now - self.last_optrange
        if st in (HoverState.TAKEOFF, HoverState.SIGN_CHECK, HoverState.HOVER, HoverState.LAND) and stale > c.stale_s and airborne:
            self._aborted = True
            self._set_state(now, HoverState.SENSOR_LOSS, f"no range data for {stale:.1f}s"); return
        if st in (HoverState.SIGN_CHECK, HoverState.HOVER) and now - self.t_state > 2.0 and airborne:
            rate, _ = self.telemetry_health(now)
            if rate < c.min_rate_inflight_hz:
                self.land(now, f"range data too slow in flight: {rate:.1f} Hz")
        if st == HoverState.SENSOR_LOSS:
            if now - self.t_state > c.sensor_loss_timeout_s:
                self._set_state(now, HoverState.DONE, "sensor loss descent finished"); return
            if now - self.t_state > c.sensor_loss_open_loop_s and stale < c.stale_s:
                self._set_state(now, HoverState.LAND, "sensors back, landing under control")
            return
        if st == HoverState.TAKEOFF and not self.lifted:
            if now - self.t_state > c.takeoff_timeout_s:
                self._set_state(now, HoverState.DONE, f"no lift-off within {c.takeoff_timeout_s:.0f} s: stopped (check props, battery, tether, ground)"); return
            if self._t_cap is not None and now - self._t_cap > c.takeoff_cap_timeout_s:
                self._set_state(now, HoverState.DONE, f"no lift-off {c.takeoff_cap_timeout_s:.0f} s after the throttle reached its cap {c.thr_max:.2f}: stopped (check props, battery, tether, ground)"); return
        if st in (HoverState.TAKEOFF, HoverState.SIGN_CHECK, HoverState.HOVER):
            if self.z > c.ceiling:
                self.land(now, f"ceiling {c.ceiling:.2f} m exceeded (z {self.z:.2f} m)")
            elif c.batt_autoland and self.batt_v < c.batt_land_v:
                self.land(now, f"battery {self.batt_v:.1f} V")
            elif self.batt_v < c.batt_land_v and "batt_low" not in self._flags:
                self._flags.add("batt_low"); self.events.append((now, f"WARNING battery {self.batt_v:.1f} V is below {c.batt_land_v:.1f} V: no automatic landing (you decide)"))
            elif st == HoverState.HOVER:
                dist = math.hypot(self.kx.x[0] - self.x_ref, self.ky.x[0] - self.y_ref)
                speed = math.hypot(self.kx.x[1], self.ky.x[1])
                if dist > c.runaway_m or speed > c.runaway_speed:
                    self.runaway_since = self.runaway_since if self.runaway_since is not None else now
                    if now - self.runaway_since > c.runaway_hold_s:
                        self.xy_disabled = True                   # stop steering: a wrong sign would keep pushing us away
                        self.land(now, "position runaway")
                else:
                    self.runaway_since = None
        if st == HoverState.TAKEOFF and c.sign_check and not self.sign_done and self.z > c.sign_height + c.sign_window_hi:
            self.sign_done = True                                # climbed through the sign-check window: skip the check, continue to the hover height (XY stays gated)
            self.events.append((now, f"sign check skipped: climbed past the window (z {self.z:.2f} > {c.sign_height + c.sign_window_hi:.2f}, vz {self.kz.x[1]:+.2f})"))
        if st == HoverState.TAKEOFF and c.sign_check and not self.sign_done and c.sign_height - 0.03 <= self.z <= c.sign_height + c.sign_window_hi and abs(self.kz.x[1]) < c.sign_max_vz:
            self.x_ref, self.y_ref = self._ref_point()
            self._base_samples = [[], []]
            self._set_state(now, HoverState.SIGN_CHECK)
        elif st == HoverState.TAKEOFF and (self.sign_done or not c.sign_check) and self.z >= c.hover_height - 0.05 and abs(self.kz.x[1]) < 0.2:
            if not c.sign_check:
                self.x_ref, self.y_ref = self.kx.x[0], self.ky.x[0]  # no test pulses: hold where we are; otherwise keep the take-off point
            self._set_state(now, HoverState.HOVER)
        if st == HoverState.SIGN_CHECK:
            e = now - self.t_state
            mean2 = [(self.flow_raw[0][a] + self.flow_raw[1][a]) / 2.0 for a in (0, 1)]    # raw sensor, not the filter (it predicts the pulse works)
            if 0.4 <= e < 1.0:
                for a in (0, 1):
                    self._base_samples[a].append(mean2[a])
            elif e >= 1.0 and self._base_samples[0] and self.sign_base == [0.0, 0.0]:
                self.sign_base = [sum(b) / len(b) for b in self._base_samples]
            for axis, (t0, t1) in enumerate(((1.0, 2.1), (2.7, 3.8))):
                if t0 <= e <= t1:                                   # best response (positive velocity change) in this axis' window
                    self.sign_peak[axis] = max(self.sign_peak[axis], mean2[axis] - self.sign_base[axis])
                if e > t1 and axis == 0 and self.sign_peak[0] < c.sign_min_response_mps:
                    self.xy_disabled = True; self.land(now, "pitch sign check failed: no forward response (wrong sign or flow sensor?)")
                if e > t1 and axis == 1 and self.sign_peak[1] < c.sign_min_response_mps:
                    self.xy_disabled = True; self.land(now, "roll sign check failed: no rightward response (wrong sign or flow sensor?)")
            if self.state == HoverState.SIGN_CHECK and e > 3.8 + 1.0:
                self.sign_done = True                                # the hold point stays where the drone TOOK OFF: it returns there after the test pulses
                drift = math.hypot(self.kx.x[0] - self.x_ref, self.ky.x[0] - self.y_ref)
                self.events.append((now, f"sign check drift: {drift:.2f} m from the take-off point (pulses {self.sign_peak[0]:+.2f} / {self.sign_peak[1]:+.2f} m/s)"))
                self._set_state(now, HoverState.TAKEOFF, "sign check passed, climbing to hover height")
        if st == HoverState.LAND:
            self._contact_s = self._contact_s + self._dt if (self.z < c.land_contact_height and self._zm_prev < 2.0 * c.land_contact_height) else 0.0      # the filtered height AND the last raw range sample agree: on the floor
            if self._ramp is not None and now - self._ramp[0] >= c.land_ramp_s:
                self._set_state(now, HoverState.DONE, "landed")
            elif self._contact_s >= c.land_contact_s and self._ramp is not None and now - self._ramp[0] >= c.land_contact_ramp_s:
                self._set_state(now, HoverState.DONE, "landed (floor contact)")        # (the throttle ramp-down on contact has run for land_contact_ramp_s: no disarm with the drone still above the floor)

    # --- control laws ----------------------------------------------------------------------
    def command_height(self, now: float, h: float) -> bool:
        """Change the hold height while flying. In HOVER the height reference glides there on a jerk-limited S-curve (speed limit min(ref_v_max, peak fraction x vz_lim)) with a velocity feed-forward: no step in the reference. During the takeoff it just changes the climb target. Refused while landing."""
        c = self.cfg
        if self.state in (HoverState.LAND, HoverState.DONE, HoverState.SENSOR_LOSS):
            return False
        c.hover_height = h
        c.ceiling = max(c.ceiling, h + 0.3)                  # raised only: lowering it under a drone that is still above would trigger a ceiling landing
        if self.state == HoverState.HOVER:
            peak = (c.climb_peak_frac if h > self.z_ref else c.descent_peak_frac) * c.vz_lim      # descending is gentler: the drone speeds up under gravity and brakes late
            self._plan(self.z_ref, self.vz_ff, h, min(peak, c.ref_v_max), ("hold", h))
        self.events.append((now, f"hold height -> {h:.2f} m"))
        return True

    def _thr_hover_now(self) -> float:
        c = self.cfg
        h0, v0 = self._hover0()
        scale = (v0 / max(self.batt_v, 1.0)) ** (2.0 / c.thrust_exp)                # thrust ~ V^2 * u^exp  =>  the throttle that still balances weight scales with V^(-2/exp); v0 is a reading of the same sensor, so a scale error cancels
        return c.thr_idle + (h0 - c.thr_idle) * min(scale, 1.6)

    def _plan(self, z0: float, v0: float, z1: float, v_max: float, tag: tuple) -> None:
        self._traj = JerkLimited1D(max(v_max, 0.02), self.cfg.ref_a_max, self.cfg.ref_j_max)
        self._traj.plan(z0, v0, z1)
        self._traj_tag = tag

    def _learn_hover(self, thr: float, dt: float) -> None:
        """Settled hover (|vz| < 0.15 m/s, within 5 cm of the reference): remember the throttle that really holds the drone; after hover_learn_min_s of it, it becomes
        the hover reference (the base throttle) instead of the lift-off value."""
        c = self.cfg
        self._learn.append((thr, self.batt_v)); self._learn_dt = dt
        if len(self._learn) * dt < c.hover_learn_min_s or len(self._learn) % 100 != 0:
            return
        h, b = self.hover_learned_raw()
        cur = self._hover0()[0]
        if abs(h - c.thr_hover0) > c.hover_learn_span:
            return                                             # not a real hover (a crawl / a bad flight): keep the reference
        if self._learn_adopted is None or abs(h - self._learn_adopted) > 0.01:
            a_old = G * (self._thrust_ratio() - 1.0)
            self.hover_ref = (h, b)
            self._learn_adopted = h
            a_new = G * (self._thrust_ratio() - 1.0)
            self.kz.x[2] += a_old - a_new                      # the model changed: keep the predicted acceleration (bias state absorbs the difference)
            self.integ = 0.0
            self.events.append((self._now, f"hover throttle from flight data: {h:.3f} at {b:.2f} V (was {cur:.3f})"))

    def hover_learned_raw(self) -> tuple[float, float]:
        xs = sorted(t for t, _ in self._learn); bs = sorted(v for _, v in self._learn)
        return xs[len(xs) // 2], bs[len(bs) // 2]

    def hover_learned_flight(self, min_s: float = 3.0) -> Optional[tuple[float, float, float]]:
        """For persisting after a flight: (hover throttle normalised to the reference voltage v0 of the configured thr_hover0 scale, pack reading, settled seconds), or None
        unless the flight settled >= min_s within 5 cm of the target and was not aborted. Same scaling as _thr_hover_now: thrust ~ V^2 u^exp."""
        c = self.cfg
        n = len(self._learn) * self._learn_dt
        if self._aborted or n < min_s or not self._learn:
            return None
        h, b = self.hover_learned_raw()
        norm = c.thr_idle + (h - c.thr_idle) * (b / max(c.batt_v0, 1.0)) ** (2.0 / c.thrust_exp)
        return norm, b, n

    def hover_learned(self) -> Optional[tuple[float, float, float]]:
        """(median hover throttle, median pack reading, seconds of settled hover behind it), or None until 5 s of settled hover. For the caller to persist: the
        throttle belongs to THAT pack reading (the reading's own scale), see _thr_hover_now for how it is used."""
        n = len(self._learn) * self._learn_dt
        if n < self.cfg.hover_learn_report_s:
            return None
        h, b = self.hover_learned_raw()
        return h, b, n

    def _velocity_loop(self, now: float, dt: float, z: float, vz: float, vz_ref: float, kv: float) -> float:
        """Climb-rate PI after lift-off. The integrator thr_i starts at the throttle that lifted the drone and converges to the throttle that holds the commanded climb
        rate = the hover throttle on THIS surface/battery, so a floor take-off (ground effect) and a box take-off (none) use the same code. Frozen on the ground, when the output
        is saturated, when the range sensor is stale; a quarter rate while the estimate does not move although more climb is wanted (cannot wind up on an estimator that is blind)."""
        c = self.cfg
        verr = vz_ref - vz
        hov = self._thr_hover_now()
        lo = max(min(hov, self.thr_at_lift) - c.vel_band_down, c.thr_min_air)
        hi = min(max(hov, self.thr_at_lift) + c.vel_band_up, c.thr_max)
        stale = now - self.last_optrange > 0.3
        stuck = abs(vz) < 0.03 and verr > 0.1
        self._stuck_t = (self._stuck_t + dt) if stuck else 0.0
        out = self.cmd.throttle if self.cmd.arm else self.thr_i
        saturated = (out >= c.thr_max - 1e-6 and verr > 0) or (out <= c.thr_min_air + 1e-6 and verr < 0)
        if z > 0.02 and not stale and not saturated:
            self._slow_t = self._slow_t + dt if verr > 0.08 else (0.0 if verr < 0.03 else self._slow_t)       # climbing slower than asked for a while: the integrator is too low (unmodelled ground effect, heavier pack): learn faster
            k = c.vel_ki * (1.0 + min(self._slow_t / 0.7, 9.0)) * (0.25 if self._stuck_t > 1.0 and verr > 0 else 1.0)
            self.thr_i += k * verr * dt
        self.thr_i = min(max(self.thr_i, lo), hi)
        cur = self.hover_ref[0] if self.hover_ref is not None else self.thr_i
        a_old = G * (self._thrust_ratio() - 1.0)
        self.hover_ref = (cur + (self.thr_i - cur) * min(dt / 0.5, 1.0), self.batt_v)         # the model's hover reference follows the integrator (= the free-air hover throttle)
        self.kz.x[2] += a_old - G * (self._thrust_ratio() - 1.0)                              # the model changed: keep the predicted acceleration
        ge = self._ge(z)
        gain = G * c.thrust_exp / max(self._hover0()[0] - c.thr_idle, 1e-6)
        return c.thr_idle + (self.thr_i - c.thr_idle) * ge ** (-1.0 / c.thrust_exp) + min(max(kv * verr, -c.vel_p_down), c.vel_p_up) + self.az_ff / gain   # ground effect and the planned acceleration as feed-forward (the integrator must not learn them)

    def _altitude(self, now: float, dt: float) -> float:
        c, st = self.cfg, self.state
        if st == HoverState.SENSOR_LOSS:
            base = self.thr_learned if self.thr_learned is not None else self._thr_hover_now() + self.integ
            return base - c.sensor_loss_thr_drop
        z, vz, bias = self.kz.x[:3]
        if c.pred_states and self.lifted and st in (HoverState.TAKEOFF, HoverState.HOVER, HoverState.SIGN_CHECK):
            dz, dvz = self._lead(now)
            z, vz = z + c.pred_gain * dz, vz + c.pred_gain * dvz
        elif c.pred_states == "air+land" and self.lifted and st == HoverState.LAND:
            vz = vz + c.pred_gain * self._lead(now)[1]                         # the descent SPEED is predicted; the height stays the estimate (the touchdown logic needs the real height)
        if st == HoverState.HOVER and self._mode_applied == "stabilize":
            self.z_ref, self.vz_ff, self._traj, self._traj_tag = z, 0.0, None, None      # no height loop: the throttle stays where it was
            self.integ = min(max(self._stab_thr - self._thr_hover_now(), -c.i_max), c.i_max)
            return self._stab_thr
        if st == HoverState.TAKEOFF:
            if self.lifted and z < 0.02 and vz < 0.05:
                if self._low_t0 is None: self._low_t0 = now
            else:
                self._low_t0 = None
            if self.lifted and self._low_t0 is not None and now - self._low_t0 >= c.lift_debounce_s:
                self._low_t0 = None
                self.lifted = False                              # sank back to the ground: keep raising the throttle from where we were
                self.thr_i = None
                self._traj = self._traj_tag = None
                self.integ = 0.0
            if not self.lifted:
                if self.ground_thr is None:
                    self.ground_thr = c.takeoff_start_frac * self._thr_hover_now()
                slow = self.ground_thr >= c.takeoff_slow_frac * self._thr_hover_now()
                rate = c.takeoff_ramp_rate
                if slow:
                    moving = z > 0.03 or vz > 0.06 or self._zm_mot > 0
                    self._nomo = max(self._nomo - 2.0 * dt, 0.0) if moving else self._nomo + dt         # time in the creep zone without any motion (decays twice as fast once the estimate moves)
                    f = min(self._nomo / max(c.takeoff_accel_s, 1e-3), 1.0); f = f * f * (3.0 - 2.0 * f)
                    rate = c.takeoff_ramp_slow + (max(c.takeoff_ramp_slow_max, c.takeoff_ramp_slow) - c.takeoff_ramp_slow) * f   # nothing happens: creep faster (a lift-off needing more than the expected throttle: boxes, no ground effect, other battery)
                if self._zm_mot > 0:
                    rate = min(rate, 0.25 * c.takeoff_ramp_slow)                                          # the range sensor already shows the drone leaving the floor: nearly hold the throttle until the lift-off is confirmed
                self._ramp_rate = rate
                cap = self._thr_hover_now() + c.takeoff_cap_above + c.takeoff_cap_relax_rate * max(now - self.t_state - c.takeoff_cap_relax_s, 0.0)
                self.ground_thr = min(self.ground_thr + rate * dt, c.thr_max, max(cap, self.ground_thr))     # raise throttle slowly until we leave the ground
                if self.ground_thr >= c.thr_max - 1e-9 and self._t_cap is None:
                    self._t_cap = now
                if (z > 0.05 and vz > 0.02) or (vz > 0.10 and z > 0.02) or self._zm_hi >= 2:
                    self._lift_cnt += 1
                else:
                    self._lift_cnt = 0
                if self._lift_cnt >= 3:                                         # 3 consecutive confirmations (~0.1 s): a single noisy reading is not a lift-off
                    self.lifted = True
                    self.t_lift = now; self.thr_at_lift = self.ground_thr
                    self.origin = (self.kx.x[0], self.ky.x[0])                    # the take-off point: the hold reference once the full XY hold opens
                    if c.ge_bonus > 0 and c.lift_ref_ge:
                        # the drone is hovering IN ground effect at the moment the lift-off is noticed (it creeps up through the 0-5 cm band until the detector fires):
                        # the free-air hover throttle follows from the same ground-effect model the height prediction uses (a fixed margin is only right at one lift-off height)
                        ge = self._ge(z)
                        self.lift_thr = min(max(c.thr_idle + (self.thr_eff_est - c.thr_idle) * ge ** (1.0 / c.thrust_exp), c.thr_idle), c.thr_max)
                    else:
                        self.lift_thr = min(max(self.ground_thr - c.takeoff_lift_margin, c.thr_idle), c.thr_max)         # the throttle that physically lifted us
                    cfg_hover = self._thr_hover_now() * min(1.0, c.thr_hover0 / max(self._hover0()[0], 1e-6))
                    self.thr_i = min(self.lift_thr - c.lift_latency_s * self._ramp_rate, cfg_hover + c.lift_start_above + c.lift_start_frac * max(self.lift_thr - cfg_hover - c.lift_start_above, 0.0))
                    if self.thr_i < self.lift_thr - c.lift_latency_s * self._ramp_rate - 1e-9:
                        self.events.append((now, f"lift-off at {self.lift_thr:.3f}: velocity loop starts at {self.thr_i:.3f} (hover about {cfg_hover:.3f}), it raises it if the drone does not climb"))
                                                          # the velocity loop's integrator starts at the throttle that lifted us (free-air equivalent: ground effect corrected)
                    if abs(self.lift_thr - self._thr_hover_now()) > c.lift_hover_tol * max(self._hover0()[0], 0.1):
                        self.events.append((now, f"lift-off at throttle {self.lift_thr:.3f}, expected about {self._thr_hover_now():.3f} (surface / battery differ: the climb velocity loop finds the hover throttle)"))
                    if c.use_lift_ref:
                        self.hover_ref = (min(self.lift_thr, c.thr_hover0 + c.hover_ref_above), self.batt_v)                                # model reference for the rest of this flight (at this pack voltage); the velocity loop keeps it current
                    a_model = G * (self._thrust_ratio() * math.cos(self.tilt_x_est) * math.cos(self.tilt_y_est) - 1.0)
                    self.integ = 0.0
                    self.kz.x[2] = -a_model + 0.3                # at lift-off the real acceleration is ~0: whatever the model predicts is its error
                    self.z_ref = max(z, 0.0)                    # the height target starts from where we really are
                    self._traj = self._traj_tag = None; self.vz_ff = min(max(vz, 0.0), c.ref_v_max)       # the reference continues the climb the drone is already in
                else:
                    self.z_ref = 0.0
                    return min(self.ground_thr, c.thr_max, c.soft_start_per_s * max(now - self.t_state, 0.0))
            cap = c.hover_height if (self.sign_done or not c.sign_check) else c.sign_height
            if c.climb_min_jerk:
                if self._traj_tag != ("up", cap):                                          # (re)start the S-curve from where the reference is now
                    self._plan(max(self.z_ref, 0.0), self.vz_ff, cap, c.ref_v_max, ("up", cap))
                self.z_ref, self.vz_ff, self.az_ff = self._traj.step(dt)
            else:
                self.z_ref = min(self.z_ref + c.takeoff_rate * dt, cap)
        elif st == HoverState.HOVER:
            if self._traj is not None and self._traj_tag[0] == "hold" and not self._traj.done:
                self.z_ref, self.vz_ff, _ = self._traj.step(dt)
            else:
                self._traj = self._traj_tag = None
                self.z_ref = c.hover_height; self.vz_ff = 0.0
        elif st == HoverState.SIGN_CHECK:
            self.z_ref = c.sign_height; self.vz_ff = 0.0; self._traj = self._traj_tag = None
        elif st == HoverState.LAND:
            band = 1 if self.z_ref <= c.land_slow_height else 0
            if self._traj_tag != ("land", band):               # jerk-limited descent: land_rate_high above land_slow_height, land_rate below; XY held; the throttle ramp takes over near the floor
                first = not (self._traj_tag and self._traj_tag[0] == "land")
                z0 = (self.z if self.z_ref > self.z + 0.2 else self.z_ref) if first else self.z_ref      # entered LAND from above the reference (or from a climb): start from where we are
                self._plan(z0, self.vz_ff, z0 - 5.0, c.land_rate if band else c.land_rate_high, ("land", band))
            self.z_ref, self.vz_ff, _ = self._traj.step(dt)
            self.z_ref = max(self.z_ref, -0.1)
        e = self.z_ref - z
        vz_ref = min(max(c.kp_z * e + self.vz_ff, -c.vz_lim), c.vz_lim)
        vel_loop = st == HoverState.TAKEOFF and self.lifted and self.thr_i is not None
        if vel_loop:
            vz_ref = min(vz_ref, c.vel_sp_factor * c.ref_v_max)       # the climb-rate setpoint never exceeds the planned speed (+10 %): no catching up on a reference that ran ahead
        if z > 0.05 and st != HoverState.TAKEOFF:                  # (during the take-off climb the velocity loop's integrator thr_i is the only integrator)
            self.integ = min(max(self.integ + c.ki_z * e * dt, -c.i_max), c.i_max)
        gain = G * c.thrust_exp / max(self._hover0()[0] - c.thr_idle, 1e-6)          # m/s^2 of vertical acceleration per unit of throttle, at hover
        if st == HoverState.LAND:
            if self._land_bias is None:
                self._land_bias = bias                           # the descent is slow and the bias observer only adds a wobble to it: keep the value it had when landing began
            bias = self._land_bias
        ff = -c.bias_ff * min(max(bias, -3.0), 3.0) / gain if (z > 0.05 and not vel_loop) else 0.0
        base = self._thr_hover_now()
        if st == HoverState.TAKEOFF and self.t_lift is not None and c.lift_blend_s > 0:
            w = min(max((now - self.t_lift) / c.lift_blend_s, 0.0), 1.0)                     # hand over from the creep throttle to the hover reference smoothly (no step)
            base = (1.0 - w) * self.thr_at_lift + w * base
        kv = c.kv_z_climb if (st == HoverState.TAKEOFF and self.lifted and c.kv_z_climb > 0) else c.kv_z
        thr = base + kv * (vz_ref - vz) + self.integ + ff
        if st == HoverState.LAND and self._ramp is None:
            self.land_i = min(max(self.land_i + c.land_ki * (vz_ref - vz) * dt, -c.land_i_max), c.land_i_max)      # descent-rate integrator: a slightly low hover reference must not turn into a fall
            thr += (c.land_kv - kv) * (vz_ref - vz) + self.land_i
            if z < c.land_slow_height + 0.15 and vz < vz_ref - 0.05:
                thr += min(c.land_brake_gain * (vz_ref - vz - 0.05), c.land_brake_max)      # sinking faster than planned near the floor: brake (one-sided, so a late estimate cannot start an oscillation upwards)
        if vel_loop:
            thr = self._velocity_loop(now, dt, z, vz, vz_ref, c.vel_kp)
        if st == HoverState.TAKEOFF and c.climb_overspeed_gain > 0.0 and self.lifted:
            thr -= c.climb_overspeed_gain * max(vz - vz_ref - c.climb_overspeed_dead, 0.0)      # climbing faster than planned (ground effect gives a burst when it fades): back off at once
        grace = c.lift_grace_m if (self.t_lift is not None and now - self.t_lift < c.lift_grace_s) else 0.0      # the lift-off is noticed late: the drone is ahead of a slow reference for a moment, do not slam the throttle for that
        if st in (HoverState.TAKEOFF, HoverState.SIGN_CHECK, HoverState.HOVER) and z > 0.05 and (z > self.z_ref + c.overshoot_m + grace or vz > c.overshoot_vz):
            if c.overshoot_soft > 0:                                          # proportional: the cut grows with the excess (full cut at (1 + overshoot_soft) x the threshold), no step for a slow 12 cm drift
                excess = max((z - self.z_ref - c.overshoot_m - grace) / c.overshoot_m, (vz - c.overshoot_vz) / c.overshoot_vz)
                cut = c.overshoot_cut * min(max(excess / c.overshoot_soft, 0.0), 1.0)
            else:
                cut = c.overshoot_cut
            thr = min(thr, self._thr_hover_now() - cut)                      # overshoot damper: climbing too high/fast -> throttle below hover
            self.integ = min(self.integ, 0.0)
            if "overshoot" not in self._flags:
                self._flags.add("overshoot"); self.events.append((self._now, f"overshoot damper (z {z:.2f}, vz {vz:+.2f})"))
        elif z < self.z_ref + c.overshoot_m * 0.5 and vz < c.overshoot_vz * 0.5:
            self._flags.discard("overshoot")
        vcap = c.vel_cap_factor * c.ref_v_max
        vchk = vz
        if st in (HoverState.TAKEOFF, HoverState.SIGN_CHECK, HoverState.HOVER) and self.lifted and vchk > vcap:
            thr -= c.vel_cap_gain * (vchk - vcap)                                # hard climb-rate limit (PX4 style): proportional cut above 1.5 x the planned climb speed
        if st in (HoverState.HOVER, HoverState.SIGN_CHECK) and abs(vz) < 0.15:       # remember the throttle that really holds us up
            self.thr_learned = thr if self.thr_learned is None else self.thr_learned + 0.02 * (thr - self.thr_learned)
        if st == HoverState.HOVER and self.lifted and abs(vz) < 0.15 and abs(z - self.z_ref) < 0.05:
            self._learn_hover(thr, dt)
        if st == HoverState.LAND:
            if self._ramp is None and z > c.land_ramp_height:
                if vz < -c.land_vz_cap:
                    thr += min(c.land_cap_gain * (-vz - c.land_vz_cap), c.land_cap_max)       # sinking faster than the cap: throttle up in proportion (like the climb cap)
                thr = max(thr, c.land_floor_frac * self._thr_hover_now() * min(1.0, c.thr_hover0 / max(self._hover0()[0], 1e-6)))                    # no free fall: never below a fraction of the (learned) hover until the last land_ramp_height
            planned_low = self.z_ref <= c.land_arm_zref                          # the planned descent has reached the floor region: a single bad range sample / estimate dip at 0.4 m must never start the ramp to idle (free fall)
            if self._ramp is None and planned_low and ((z <= c.land_ramp_height and self._zm_prev <= 2.0 * c.land_ramp_height) or self._contact_s >= c.land_contact_s):
                self._ramp = (now, thr)                                   # from the current throttle ...
                self.events.append((now, f"touchdown ramp: z {z:.2f} (raw {self._zm_prev:.2f}), z_ref {self.z_ref:.2f}, vz {vz:+.2f}, contact {self._contact_s:.2f} s"))
            if self._ramp is not None:
                t0, thr0 = self._ramp                                     # ... linearly to idle
                ramp = thr0 + (c.thr_idle - thr0) * min(max((now - t0) / c.land_ramp_s, 0.0), 1.0)
                if now - t0 < c.land_touch_hold_s:
                    ramp = max(ramp, c.land_touch_frac * self._thr_hover_now())             # the height near the floor is uncertain by a few cm: for the first half second the thrust stays close to the weight, so a not-yet-touched drone settles instead of dropping
                thr = ramp if vz > -c.land_sink_guard else max(ramp, thr)  # soft touchdown: sinking too fast keeps the altitude loop as a floor
        return thr

    def _ref_point(self) -> tuple:
        """Hold reference: the take-off point if the estimate since lift-off is plausible (< xy_origin_max_m away), else where we are now (no jump)."""
        cur = (self.kx.x[0], self.ky.x[0])
        o = self.origin
        if o is not None and math.hypot(cur[0] - o[0], cur[1] - o[1]) < self.cfg.xy_origin_max_m:
            return o
        return cur

    def _xy_gate_update(self) -> None:
        """XY hold only acts when the horizontal estimate is trustworthy (see HoverConfig.xy_*). Two gates: damping (velocity only, from xy_damp_min_height_m) and full hold (from xy_min_height_m)."""
        c, now = self.cfg, self._now
        speed = math.hypot(self.kx.x[1], self.ky.x[1])
        if speed > c.xy_max_speed_ms:
            self._xy_fast_until = now + c.xy_speed_hold_s
        thr_h = c.xy_min_height_m - (c.xy_height_hyst_m if self._xy_gate else 0.0)
        thr_d = c.xy_damp_min_height_m - (c.xy_damp_hyst_m if self._damp_gate else 0.0)
        if not self.lifted: why = why_d = "on the floor"
        elif now - self._flow_t > c.xy_flow_max_age_s: why = why_d = "flow data stale"
        elif self._flow_q < c.xy_min_flow_q: why = why_d = f"flow quality {self._flow_q} below {c.xy_min_flow_q}"
        elif now < self._xy_fast_until: why = why_d = f"horizontal speed {speed:.2f} m/s above {c.xy_max_speed_ms:.2f}"
        else: why = why_d = ""
        if why_d == "" and self.z < thr_d: why_d = f"height {self.z:.2f} m below {thr_d:.2f} m"
        if why == "" and self.z < thr_h: why = f"height {self.z:.2f} m below {thr_h:.2f} m"
        if (why_d == "") != self._damp_gate:
            self._damp_gate = why_d == ""
            self.events.append((now, "XY damping opened: velocity feedback only" if self._damp_gate else f"XY damping closed: {why_d}"))
        if (why == "") != self._xy_gate:
            self._xy_gate = why == ""
            self.events.append((now, "XY gate opened: hold here" if self._xy_gate else f"XY gate closed: {why}"))
            if self._xy_gate:
                self.x_ref, self.y_ref = self._ref_point()
                al = G * math.tan(math.radians(c.xy_i_max_deg))
                self.ix = [min(max(l / c.ki_xy, -al / c.ki_xy), al / c.ki_xy) if c.ki_xy > 0 else 0.0 for l in self._lean]     # the push learned while only damping becomes the integral term
        self._xy_gate_reason = why or "open"

    def _horizontal(self, dt: float) -> tuple[float, float]:
        c = self.cfg
        if self._mode_applied != "pos_hold" and self.state not in (HoverState.TAKEOFF, HoverState.SIGN_CHECK):
            step = c.mode_tilt_slew_deg_s * dt / c.angle_limit_deg                 # XY hold is off: the tilt command goes to zero without a step and stays there
            self._out = [min(max(0.0, o - step), o + step) if abs(o) > step else 0.0 for o in self._out]
            self.ix = [0.0, 0.0]; self._lean = [0.0, 0.0]; self.x_ref, self.y_ref = self.kx.x[0], self.ky.x[0]
            return self._out[0], self._out[1]
        self._xy_gate_update()
        landing = self.state == HoverState.LAND
        pre_sign = self.state == HoverState.TAKEOFF and c.sign_check and not self.sign_done
        damp = self._damp_gate and not (landing and self.z < 0.15)         # no damping-only steering in the last 15 cm of a landing
        full = self._xy_gate and not pre_sign
        if self.xy_disabled or self.state in (HoverState.SENSOR_LOSS, HoverState.DONE) or not damp:
            self.ix = [0.0, 0.0]; self._w = 0.0; self._out = [0.0, 0.0]; self._lean = [0.0, 0.0]
            if not self._xy_gate: self.x_ref, self.y_ref = self.kx.x[0], self.ky.x[0]
            return 0.0, 0.0
        if self.state == HoverState.SIGN_CHECK:                      # small test pulses: forward, then right; the supervisor checks the response
            e = self._now - self.t_state
            lim = c.sign_pulse_deg / c.angle_limit_deg
            if 1.0 <= e < 1.5:                                    # open loop on purpose: feedback would hide a wrong sign
                return lim, 0.0
            if 1.5 <= e < 2.1 or 3.2 <= e < 3.8:
                return 0.0, 0.0                                   # coast and measure
            if 2.7 <= e < 3.2:
                return 0.0, lim
            # outside the measuring windows: normal position hold, so the drone brakes and returns to where it was
        if not full:
            self.x_ref, self.y_ref = self.kx.x[0], self.ky.x[0]       # no phantom position error while only damping
            self.ix = [0.0, 0.0]
        self._w = min(self._w + dt / max(c.xy_blend_s, 1e-3), 1.0) if full else max(self._w - dt / max(c.xy_blend_s, 1e-3), 0.0)
        w = self._w
        out = []
        a_lim = G * math.tan(math.radians(c.xy_i_max_deg))             # the integral may contribute at most xy_i_max_deg of tilt
        if landing:
            self.ix = [0.0, 0.0]                                       # reset once landing starts, no integration
        t_damp = math.radians(c.xy_damp_tilt_max_deg)
        tmax = t_damp + w * (math.radians(c.tilt_max_deg) - t_damp)    # authority fades from the damping limit to the full tilt limit
        kv = c.kv_xy_damp + w * (c.kv_xy - c.kv_xy_damp)
        for i, (kf, ref) in enumerate(((self.kx, self.x_ref), (self.ky, self.y_ref))):
            x, v = kf.x
            e = ref - x
            v_ref = w * min(max(c.kp_pos * e, -c.v_lim), c.v_lim)
            a = kv * (v_ref - v)
            if not full and abs(v) < 0.3:
                self._lean[i] = min(max(self._lean[i] - c.xy_damp_ki * v * dt, -a_lim), a_lim)      # learn the constant push from the residual velocity (damping zone only)
            a_i = w * min(max(c.ki_xy * self.ix[i], -a_lim), a_lim) + (1.0 - w) * self._lean[i]
            tilt_raw = math.atan2(a + a_i, G)
            tilt = min(max(tilt_raw, -tmax), tmax)
            if not landing and full and w >= 1.0 and c.ki_xy > 0.0 and abs(tilt_raw) < tmax and abs(e) < c.xy_i_range_m:    # conditional anti-windup: only while the command is not saturated and the error is small
                self.ix[i] = min(max(self.ix[i] + e * dt, -a_lim / c.ki_xy), a_lim / c.ki_xy)
            o = tilt / math.radians(c.angle_limit_deg)
            if w < 1.0:                                                # damping / blending: no tilt step faster than xy_slew_deg_s
                step = c.xy_slew_deg_s * dt / c.angle_limit_deg
                o = min(max(o, self._out[i] - step), self._out[i] + step)
            self._out[i] = o
            out.append(o)
        px, ry = out
        return px, ry                                    # physical sticks; hardware signs are applied in update()
