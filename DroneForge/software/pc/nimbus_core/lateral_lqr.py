"""Nimbus-style lateral control (DfLqr structure, docs/nimbus_analysis/03 section 1.2) for our drone. Qt-free, ROS-free.

Per lateral axis (pitch lane along the yaw-aligned forward unit vector, roll lane along the right unit vector):
    I      += dt * e_p ; I clamped to +-(I_lim_deg / Ki)
    u_raw   = Ki*I + Kp*e_p + Kv*e_v - Kw*omega_deg                          [degrees of tilt]
    u       = clamp(u_raw, +-max_tilt_deg); if |accel vector| > a_max it is scaled down; conditional integration (anti-windup)
The FC runs in ANGLE mode: the tilt is converted to a stick value with the inverse of the angle curve  deg = 60 * (70|s| + 600 s^2) / 670  (Nimbus CrsfMapper).
Gains are NOT Nimbus' Air75 numbers: they are computed for OUR plant (tilt-to-acceleration K and drag D from the identification) so that the closed loop has the
same shape Nimbus gets (natural frequency ~1.4 rad/s, damping ~0.77): Kp = wn^2/K_deg, Kv = (2 zeta wn - D)/K_deg, Ki = 0.316, Kw = 0.1 (Nimbus values for Ki, Kw).
"""
from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass, field

import numpy as np


def deg_to_stick(deg: float, max_deg: float = 60.0) -> float:
    """Inverse of  deg = 60*(70|s|+600 s^2)/670 ."""
    a = abs(deg) * 670.0 / 60.0
    s = (math.sqrt(4900.0 + 4 * 600.0 * a) - 70.0) / 1200.0
    return math.copysign(min(s, 1.0), deg)


def stick_to_deg(s: float) -> float:
    s = max(-1.0, min(1.0, s)); return math.copysign(60.0 * (70.0 * abs(s) + 600.0 * s * s) / 670.0, s)


@dataclass
class PlantID:
    """Identified tilt-to-motion plant of our drone (config/plant.json, written by tools/sysid_plant.py). Defaults = first estimate from the sim step test."""
    k_f: float = 9.81; k_r: float = 9.81        # m/s^2 per rad
    d_f: float = 0.4; d_r: float = 0.4          # 1/s
    delay_s: float = 0.15                       # command-to-motion dead time

    @staticmethod
    def load(path: str) -> "PlantID":
        try: return PlantID(**{k: float(v) for k, v in json.load(open(path)).items() if k in PlantID.__dataclass_fields__})
        except Exception: return PlantID()


@dataclass
class LqrConfig:
    wn: float = 1.1; zeta: float = 0.77; ki: float = 0.316; kw: float = 0.02      # Nimbus uses 0.10; with the ~0.1 s tilt lag of our drone that rate feedback made the hold shake (3x in the model, 2 deg in Gazebo)
    max_tilt_deg: float = 15.0; a_max: float = 3.0; i_lim_deg: float = 5.0
    v_lpf_hz: float = 3.0           # low-pass on the velocity feedback: the flow-based velocity is noisy and Kv would turn that noise into tilt shake
    slew_deg_s: float = 60.0         # limit on how fast the tilt command may change


class LateralLqr:
    def __init__(self, plant: PlantID | None = None, cfg: LqrConfig | None = None) -> None:
        self.plant = plant or PlantID(); self.cfg = cfg or LqrConfig(); c = self.cfg
        kdf = self.plant.k_f * math.pi / 180.0; kdr = self.plant.k_r * math.pi / 180.0
        self.kdeg = (kdf, kdr)
        self.kp = (c.wn ** 2 / kdf, c.wn ** 2 / kdr); self.kv = (max(0.0, 2 * c.zeta * c.wn - self.plant.d_f) / kdf, max(0.0, 2 * c.zeta * c.wn - self.plant.d_r) / kdr)
        self.I = [0.0, 0.0]; self.last_u = (0.0, 0.0); self.vflt = None; self.u_prev = (0.0, 0.0)

    def reset(self) -> None:
        self.I = [0.0, 0.0]; self.vflt = None; self.u_prev = (0.0, 0.0)

    def step(self, dt: float, yaw: float, p, p_ref, v, v_ref, omega_fr) -> tuple[float, float]:
        """p, p_ref, v, v_ref: world 2-vectors (m, m/s); omega_fr = (pitch rate, roll rate) rad/s. Returns (pitch_deg forward +, roll_deg right +)."""
        c = self.cfg; f = (math.cos(yaw), math.sin(yaw)); r = (math.sin(yaw), -math.cos(yaw))
        if c.v_lpf_hz > 0:
            a = dt / (dt + 1.0 / (2 * math.pi * c.v_lpf_hz)); self.vflt = (v[0], v[1]) if self.vflt is None else (self.vflt[0] + a * (v[0] - self.vflt[0]), self.vflt[1] + a * (v[1] - self.vflt[1])); v = self.vflt
        ep = (p_ref[0] - p[0], p_ref[1] - p[1]); ev = (v_ref[0] - v[0], v_ref[1] - v[1])
        e_p = (ep[0] * f[0] + ep[1] * f[1], ep[0] * r[0] + ep[1] * r[1]); e_v = (ev[0] * f[0] + ev[1] * f[1], ev[0] * r[0] + ev[1] * r[1])
        om = (math.degrees(omega_fr[0]), math.degrees(omega_fr[1])); u = [0.0, 0.0]; lim = c.i_lim_deg / max(c.ki, 1e-6)
        for a in (0, 1):
            Inew = float(np.clip(self.I[a] + dt * e_p[a], -lim, lim))
            raw = c.ki * Inew + self.kp[a] * e_p[a] + self.kv[a] * e_v[a] - c.kw * om[a]
            u[a] = float(np.clip(raw, -c.max_tilt_deg, c.max_tilt_deg))
            if not (raw != u[a] and (raw - u[a]) * e_p[a] > 0): self.I[a] = Inew                  # conditional integration: hold the integral while saturated and still pushing
        acc = math.hypot(u[0] * self.kdeg[0], u[1] * self.kdeg[1])
        if acc > c.a_max: s = c.a_max / acc; u = [u[0] * s, u[1] * s]
        if c.slew_deg_s > 0: step = c.slew_deg_s * dt; u = [self.u_prev[0] + float(np.clip(u[0] - self.u_prev[0], -step, step)), self.u_prev[1] + float(np.clip(u[1] - self.u_prev[1], -step, step))]
        self.u_prev = (u[0], u[1]); self.last_u = (u[0], u[1]); return u[0], u[1]
