"""Small 2D-plus-altitude drone simulator with the data path we measured on the real link.

What it models (all values are assumptions to be replaced with flight data, see CALIBRATION in the docs):
- vertical: thrust acceleration proportional to (throttle - idle), motor lag, drag, battery sag (thrust ~ V^2), ground contact
- horizontal: Betaflight ANGLE mode: stick -> tilt angle with a first-order response, acceleration = g * tan(tilt) - drag * v
- sensors: range (+cos(tilt) error) and optical-flow velocity at ~10 Hz, attitude at ~3.6 Hz, noise, 80-150 ms delay, drops, optional gaps
- control path: commands applied after a fixed 22 ms delay (measured PC -> FC latency)
Axes: x forward, y right, z up. Positive pitch command = accelerate forward (nose down), positive roll command = accelerate right.
"""
from __future__ import annotations

import heapq
import math
import random
from dataclasses import dataclass, field
from typing import Optional

G = 9.81


@dataclass
class Sample:
    t_arrival: float
    kind: str                           # "optrange" | "attitude" | "battery"
    data: dict


@dataclass
class SimParams:
    hover_thr: float = 0.42            # throttle stick that hovers at full battery (UNKNOWN on the real drone)
    thr_idle: float = 0.05
    thrust_exp: float = 1.0            # thrust ~ (throttle above idle)^exp: 1 = linear (old default), ~2 = what a real prop/motor does
    mass_factor: float = 1.0           # >1 = heavier than assumed, needs more throttle
    drag: float = 0.3                  # 1/s linear drag
    tau_motor: float = 0.06
    tau_att: float = 0.08              # ANGLE-mode attitude response time constant
    angle_limit_deg: float = 60.0      # Betaflight angle_limit: stick 1.0 = this many degrees
    pitch_sign: float = 1.0            # hardware sign (+1: positive stick -> forward accel). flip to test a wiring/sign mistake
    roll_sign: float = 1.0
    batt_v0: float = 16.2
    batt_sag_per_s: float = 0.01       # V/s
    batt_read_scale: float = 1.0       # the FC reads the pack this fraction of the true voltage (real incident 2026-11-06: ~0.82); thrust still follows the TRUE voltage
    ctrl_delay_s: float = 0.022
    optrange_hz: float = 10.0
    attitude_hz: float = 3.6
    sensor_delay: tuple = (0.08, 0.15)
    sensor_drop: float = 0.03
    range_sigma: float = 0.01
    floor_steps: list = field(default_factory=list)      # [(t_start, t_end, height_m)] a box/rug under the drone: the floor is height_m higher in that window
    range_spikes: list = field(default_factory=list)     # [(t_start, t_end, value_m)] the sensor reports this value (0.0 = the invalid reading the real one gives when tilted)
    imu_hz: float = 0.0                 # >0: the FC also sends the WINDOW-MEAN vertical acceleration (world, g removed) with each sample at this rate
    imu_sigma: float = 0.25             # m/s^2 noise on that mean (vibration, accel calibration error)
    imu_bias: float = 0.15              # m/s^2 constant accel error
    ground_range_m: float = 0.0         # what the range sensor reads when the drone sits on the floor (measured on the real drone: 0.04-0.08 m)
    range_low_bias_m: float = 0.0       # the real MTF-02P reads this much too HIGH below ~32 cm (measured: +9 cm at 20 cm, +8 at 30, ~0 at 50), fading out by 45 cm
    ground_effect_bonus: float = 0.0    # extra thrust fraction on the floor (0 = off); fades out linearly to 0 at ground_effect_h_m
    ground_effect_h_m: float = 0.2
    range_dropout_p: float = 0.0        # probability that a range sample while airborne reads the floor (~0.01 m)
    flow_sigma: float = 0.03
    flow_spike_p: float = 0.0           # probability that a flow sample below flow_noise_h_m is a heavy-tailed spike (real flow near the floor: spikes of 1-2 m/s, quality 40-130)
    flow_spike_mag: tuple = (1.0, 2.0)  # m/s, random direction
    flow_spike_q: tuple = (40, 130)     # quality reported with a spike
    flow_noise_low: float = 0.0         # extra flow noise (m/s, 1 sigma) below flow_noise_h_m
    flow_noise_h_m: float = 0.3
    attitude_sigma_deg: float = 0.3
    gaps: list = field(default_factory=list)       # [(t_start, t_end)] windows where NO telemetry arrives
    wind: list = field(default_factory=list)       # [(t_start, t_end, ax, ay)] extra horizontal acceleration m/s^2


class DroneSim:
    def __init__(self, p: Optional[SimParams] = None, seed: int = 1):
        self.p = p or SimParams()
        self.rng = random.Random(seed)
        self.t = 0.0
        self.z = self.vz = 0.0
        self.x = self.y = self.vx = self.vy = 0.0
        self.tilt_x = self.tilt_y = 0.0           # rad, about each horizontal axis (+ = accelerate forward / right)
        self.thr_eff = 0.0
        self.batt_v = self.p.batt_v0
        self.armed = False
        self._cmd = (0.0, 0.0, 0.0, False)         # delayed command pipeline: roll, pitch, throttle, arm
        self._pipe: list = []
        self._next = {"optrange": 0.0, "attitude": 0.05, "imu": 0.02}
        self._inbox: list = []                    # heap of (t_arrival, seq, Sample)
        self._seq = 0
        self.trace = []; self.impact_trace = None
        self.crashed = False; self.impact_t = None; self.impact_thr = None
        self.max_impact = 0.0
        self._az_sum = 0.0; self._az_n = 0

    # --- helpers ---------------------------------------------------------------------------
    def _in_gap(self, t: float) -> bool:
        return any(a <= t < b for a, b in self.p.gaps)

    def _emit(self, kind: str, data: dict) -> None:
        if self._in_gap(self.t) or self.rng.random() < self.p.sensor_drop:
            return
        arr = self.t + self.rng.uniform(*self.p.sensor_delay)
        if self._in_gap(arr):
            return
        self._seq += 1
        heapq.heappush(self._inbox, (arr, self._seq, Sample(arr, kind, data)))

    def poll(self) -> list[Sample]:
        """Samples that have arrived at the PC by now (what the telemetry reader would hand over)."""
        out = []
        while self._inbox and self._inbox[0][0] <= self.t:
            out.append(heapq.heappop(self._inbox)[2])
        return out

    # --- physics ---------------------------------------------------------------------------
    def step(self, dt: float, roll: float, pitch: float, throttle: float, arm: bool) -> None:
        p = self.p
        self._pipe.append((self.t + p.ctrl_delay_s, (roll, pitch, throttle, arm)))
        while self._pipe and self._pipe[0][0] <= self.t:
            self._cmd = self._pipe.pop(0)[1]
        roll_c, pitch_c, thr_c, arm_c = self._cmd
        self._tn = getattr(self, '_tn', 0) + 1
        if self._tn % 10 == 0:
            self.trace.append((round(self.t, 2), round(self.z, 3), round(self.vz, 2), round(thr_c, 3), round(self.thr_eff, 3)))
            if len(self.trace) > 160: self.trace.pop(0)
        self.armed = bool(arm_c)
        thr = thr_c if self.armed else 0.0
        self.thr_eff += (thr - self.thr_eff) * dt / p.tau_motor
        self.batt_v = max(p.batt_v0 - p.batt_sag_per_s * self.t, 9.0)
        # vertical
        u = max(self.thr_eff - p.thr_idle, 0.0) / max(p.hover_thr - p.thr_idle, 1e-6)
        thrust = (u ** p.thrust_exp) * (self.batt_v / p.batt_v0) ** 2 / p.mass_factor
        if p.ground_effect_bonus:
            thrust *= 1.0 + p.ground_effect_bonus * max(0.0, 1.0 - self.z / p.ground_effect_h_m)
        az = G * (thrust * math.cos(self.tilt_x) * math.cos(self.tilt_y) - 1.0)   # thrust only counts when armed (thr = 0 otherwise)
        # horizontal attitude (ANGLE mode)
        tx_c = math.radians(p.angle_limit_deg) * pitch_c * p.pitch_sign if self.armed else 0.0
        ty_c = math.radians(p.angle_limit_deg) * roll_c * p.roll_sign if self.armed else 0.0
        self.tilt_x += (tx_c - self.tilt_x) * dt / p.tau_att
        self.tilt_y += (ty_c - self.tilt_y) * dt / p.tau_att
        ax = G * math.tan(self.tilt_x) - p.drag * self.vx
        ay = G * math.tan(self.tilt_y) - p.drag * self.vy
        for a, b, wx, wy in p.wind:
            if a <= self.t < b:
                ax += wx; ay += wy
        az -= 0.15 * self.vz
        self._az_sum += az; self._az_n += 1
        on_ground = self.z <= 0.0 and self.vz <= 0.0 and az <= 0.0
        if on_ground:
            self.z = 0.0; self.vz = 0.0; ax = ay = 0.0; self.vx *= 0.5; self.vy *= 0.5
        else:
            self.vz += az * dt
            self.vx += ax * dt; self.vy += ay * dt
            self.z += self.vz * dt; self.x += self.vx * dt; self.y += self.vy * dt
            if self.z < 0.0:
                if -self.vz > 2.0 and self.impact_trace is None: self.impact_trace = list(self.trace)
                if -self.vz > self.max_impact: self.impact_t = self.t; self.impact_thr = self.thr_eff
                self.max_impact = max(self.max_impact, -self.vz)
                if -self.vz > 2.0:
                    self.crashed = True
                self.z = 0.0; self.vz = 0.0
        if abs(self.tilt_x) > math.radians(75) or abs(self.tilt_y) > math.radians(75):
            self.crashed = True
        self.t += dt
        # sensors
        if self.t >= self._next["optrange"]:
            self._next["optrange"] += 1.0 / p.optrange_hz
            tilt = math.sqrt(self.tilt_x ** 2 + self.tilt_y ** 2)
            floor = sum(h for a, b, h in p.floor_steps if a <= self.t < b)
            rng = (self.z - floor) / max(math.cos(self.tilt_x) * math.cos(self.tilt_y), 0.3) + p.ground_range_m + self.rng.gauss(0, p.range_sigma)
            shift = max(p.ground_range_m - 0.07, 0.0)       # a drone resting higher above the floor (on boxes) reads more at rest: the sensor's bias curve belongs to the height above that ground, like the controller's calibration
            rb = rng - shift
            if p.range_low_bias_m and rb < 0.45:                                    # 0 on the ground, up to the full bias by 20 cm, held to 30 cm, gone by 45 cm
                rng += p.range_low_bias_m * (min(max((rb - 0.06) / 0.14, 0.0), 1.0) if rb < 0.32 else max(0.0, (0.45 - rb) / 0.13))
            if p.range_dropout_p and self.z > 0.04 and self.rng.random() < p.range_dropout_p:
                rng = 0.01
            for a, b, v in p.range_spikes:
                if a <= self.t < b: rng = v
            fvx, fvy, fq = self.vx + self.rng.gauss(0, p.flow_sigma), self.vy + self.rng.gauss(0, p.flow_sigma), 200 if self.z > 0.04 else 0
            if (p.flow_noise_low or p.flow_spike_p) and 0.04 < self.z < p.flow_noise_h_m:
                fvx += self.rng.gauss(0, p.flow_noise_low); fvy += self.rng.gauss(0, p.flow_noise_low)
                if p.flow_spike_p and self.rng.random() < p.flow_spike_p:
                    m, ang = self.rng.uniform(*p.flow_spike_mag), self.rng.uniform(0, 2 * math.pi)
                    fvx += m * math.cos(ang); fvy += m * math.sin(ang); fq = self.rng.randint(*p.flow_spike_q)
            self._emit("optrange", {"distance_m": max(rng, 0.0), "flow_vx": fvx, "flow_vy": fvy, "quality": fq})
        if p.imu_hz > 0 and self.t >= self._next["imu"]:
            self._next["imu"] += 1.0 / p.imu_hz
            n = max(self._az_n, 1)
            self._emit("imu", {"az_mean": self._az_sum / n + p.imu_bias + self.rng.gauss(0, p.imu_sigma), "window_s": n * 0.01})
            self._az_sum = 0.0; self._az_n = 0
        if self.t >= self._next["attitude"]:
            self._next["attitude"] += 1.0 / p.attitude_hz
            s = p.attitude_sigma_deg
            self._emit("attitude", {"pitch_deg": math.degrees(self.tilt_x) + self.rng.gauss(0, s),
                                    "roll_deg": math.degrees(self.tilt_y) + self.rng.gauss(0, s), "yaw_deg": 0.0})
            self._emit("battery", {"voltage_v": self.batt_v * p.batt_read_scale + self.rng.gauss(0, 0.02)})
