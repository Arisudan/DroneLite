"""State estimation for our drone, ported from the structure of the Nimbus DfKalman2 lateral filter (docs/nimbus_analysis/04, section 1) and a small vertical filter.
Qt-free, ROS-free. NOT a copy of Nimbus' constants: the process-model gains (tilt-to-acceleration K, drag D) belong to the plant and are set from our own
identification (config/plant.json from tools/sysid_plant.py); the structure, the update order, the noise formulas, the NIS gate and the correction limit follow Nimbus.

LateralKF (12 of Nimbus' 14 states; the two slow trim-angle states are left out):
  0,1 position x,y (world, relative to the arm point)    2,3 velocity x,y (world)    4,5 roll, pitch (measured by the FC attitude)
  6,7 body rate about x (roll rate), about y (pitch rate)    8,9 gyro bias x,y    10,11 lateral acceleration disturbance (world)
Yaw is an external input (FC attitude), not a state, exactly as in Nimbus.
Process model while flying: world acceleration = R(yaw) * [ Kf*pitch - Df*v_fwd ,  -(Kr*roll - Dr*v_right) ] + disturbance  (FLU frame: pitch + = forward, roll + = right).
Updates (each scalar, NIS gate y^2/S <= 10.8): attitude, gyro (rate + bias), optical flow (v_fwd, v_right in body axes, with the gyro rotation term).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

NIS_GATE = 10.8


@dataclass
class Plant:
    k_f: float = 9.81      # m/s^2 per rad of pitch
    k_r: float = 9.81      # m/s^2 per rad of roll
    d_f: float = 0.4       # 1/s drag, forward
    d_r: float = 0.4       # 1/s drag, sideways


@dataclass
class FlowParams:
    r0: float = 0.01           # (m/s)^2 base variance of the flow velocity (Nimbus: 0.25 for its sensor; ours is set from the MTF-02P noise)
    rot_comp: float = 0.0      # gyro rotation term factor; Nimbus uses 0.758 with its sensor, 0.0 assumes the sensor compensates its own rotation
    max_speed: float = 6.0     # reject flow above this (m/s)
    min_quality: int = 30      # reject flow below this quality (0..255)
    pos_corr_limit: float = 1.0  # m: if a flow update would move the position by more, only the velocity part is applied


class LateralKF:
    N = 12

    def __init__(self, plant: Plant | None = None, flow: FlowParams | None = None, att_var: float = 0.0027416, gyro_var: float = 0.043865, q_scale: float = 1.0) -> None:
        self.q_scale = q_scale; self.plant = plant or Plant(); self.fp = flow or FlowParams(); self.att_var = att_var; self.gyro_var = gyro_var; self.log_nis = False; self.nis: dict = {}
        self.x = np.zeros(self.N); self.P = np.diag([0.01, 0.01, 0.05, 0.05, 0.01, 0.01, 0.1, 0.1, 0.01, 0.01, 0.1, 0.1])
        self.yaw = 0.0; self.t = None; self.flying = False; self.rejected = 0; self.accepted = 0; self.flow_rejects_in_a_row = 0

    # ---- model
    def _f(self, x: np.ndarray, dt: float, yaw: float, flying: bool) -> np.ndarray:
        y = x.copy(); c, s = math.cos(yaw), math.sin(yaw)
        if flying:
            vf = c * x[2] + s * x[3]; vr = s * x[2] - c * x[3]
            af = self.plant.k_f * x[5] - self.plant.d_f * vf; ar = self.plant.k_r * x[4] - self.plant.d_r * vr
            ax = c * af - s * (-ar) + x[10]; ay = s * af + c * (-ar) + x[11]               # body left = -right
            y[0] += x[2] * dt + 0.5 * ax * dt * dt; y[1] += x[3] * dt + 0.5 * ay * dt * dt; y[2] += ax * dt; y[3] += ay * dt
        return y

    def _Q(self, dt: float, flying: bool) -> np.ndarray:
        q = np.array([0.25 * dt * dt, 0.25 * dt * dt, 1.2 * dt, 1.2 * dt, 0.001225 * dt, 0.001225 * dt, 1.0 * dt, 1.0 * dt, 0.0004 * dt, 0.0004 * dt, 0.0064 * dt, 0.0064 * dt])
        q[[2, 3, 10, 11]] *= self.q_scale                                  # velocity / acceleration-disturbance process noise: tuned from real data (tools_scripts/calibrate_kf.py)
        if not flying: q[[0, 1, 2, 3, 10, 11]] = 0.0
        return np.diag(q)

    def predict(self, t: float, yaw: float, flying: bool) -> None:
        self.yaw = yaw; self.flying = flying
        if self.t is None: self.t = t; return
        dt = min(max(t - self.t, 0.0), 0.2); self.t = t
        if dt <= 0: return
        F = np.eye(self.N); eps = 1e-5; f0 = self._f(self.x, dt, yaw, flying)
        for i in range(self.N):                                         # numerical Jacobian (12 states, cheap)
            xp = self.x.copy(); xp[i] += eps; F[:, i] = (self._f(xp, dt, yaw, flying) - f0) / eps
        self.x = f0; self.P = F @ self.P @ F.T + self._Q(dt, flying)

    # ---- scalar update with the Nimbus NIS gate
    def _update(self, H: np.ndarray, z: float, R: float, pos_limit: bool = False, tag: str = "") -> bool:
        R = max(R, 1e-7); S = float(H @ self.P @ H) + R; y = z - float(H @ self.x)
        if self.log_nis: self.nis.setdefault(tag, []).append(y * y / S)               # normalised innovation squared: mean ~ 1 when R and Q are right
        if y * y / S > NIS_GATE: self.rejected += 1; return False
        K = (self.P @ H) / S; dx = K * y
        if pos_limit and math.hypot(dx[0], dx[1]) > self.fp.pos_corr_limit: dx[0] = dx[1] = 0.0
        self.x = self.x + dx; I = np.eye(self.N) - np.outer(K, H); self.P = I @ self.P @ I.T + R * np.outer(K, K); self.accepted += 1
        return True

    def update_attitude(self, roll: float, pitch: float, var: float | None = None) -> None:
        var = self.att_var if var is None else var
        for i, z in ((4, roll), (5, pitch)):
            H = np.zeros(self.N); H[i] = 1.0; self._update(H, z, var, tag="attitude")

    def update_gyro(self, gx: float, gy: float, var: float | None = None) -> None:
        var = self.gyro_var if var is None else var
        for i, z in ((6, gx), (7, gy)):
            H = np.zeros(self.N); H[i] = 1.0; H[i + 2] = 1.0; self._update(H, z, var, tag="gyro")          # measurement = rate + bias

    def update_flow(self, v_fwd: float, v_right: float, height: float, quality: int = 255) -> bool:
        """Optical flow velocity in body axes (m/s, already scaled by the sensor) and the range-sensor height."""
        fp = self.fp
        if not self.flying or height <= 0 or not math.isfinite(height) or quality < fp.min_quality: return False
        if abs(v_fwd) > fp.max_speed or abs(v_right) > fp.max_speed: return False
        c, s = math.cos(self.yaw), math.sin(self.yaw); ok = False
        om_f, om_r = self.x[7], self.x[6]                                                       # pitch rate drives the forward lane, roll rate the sideways lane
        vbt_f = v_fwd + fp.rot_comp * om_f * height; vbt_r = v_right + fp.rot_comp * om_r * height
        Hf = np.zeros(self.N); Hf[2] = c; Hf[3] = s                                              # v_fwd   = cos*vx + sin*vy
        Hr = np.zeros(self.N); Hr[2] = s; Hr[3] = -c                                             # v_right = sin*vx - cos*vy
        for H, z, om in ((Hf, vbt_f, om_f), (Hr, vbt_r, om_r)):
            R = (0.0025 * (fp.rot_comp * om) ** 2 + fp.r0 + self.P[7, 7] * (fp.rot_comp * height) ** 2)         # Nimbus variance form
            ok |= self._update(H, z, R, pos_limit=True, tag="flow")
        # Lockout protection: if the model is wrong (the drone is blocked by a wall, pushed by a gust) the NIS gate would reject every flow sample from then on.
        # After ~0.3 s of consecutive rejections the velocity and acceleration-disturbance covariances are inflated so the sensor is trusted again (Nimbus inflates by rate / quality scale).
        self.flow_rejects_in_a_row = 0 if ok else self.flow_rejects_in_a_row + 1
        if self.flow_rejects_in_a_row >= 8:
            self.P[2, 2] += 1.0; self.P[3, 3] += 1.0; self.P[10, 10] += 1.0; self.P[11, 11] += 1.0; self.flow_rejects_in_a_row = 0
        return ok

    def update_position(self, x: float, y: float, var: float, tag: str = "vo") -> bool:
        """Absolute position measurement (visual odometry against a keyframe), world frame, same NIS gate."""
        ok = False
        for i, z in ((0, x), (1, y)):
            H = np.zeros(self.N); H[i] = 1.0; ok |= self._update(H, z, var, tag=tag)
        return ok

    # ---- outputs
    @property
    def pos(self): return float(self.x[0]), float(self.x[1])

    @property
    def vel_world(self): return float(self.x[2]), float(self.x[3])

    def vel_body(self):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return c * self.x[2] + s * self.x[3], -(s * self.x[2] - c * self.x[3])                    # forward, LEFT

    def rates(self): return float(self.x[6]), float(self.x[7])


class VerticalKF:
    """[z, vz] from the range sensor (tilt-compensated) with a constant-velocity model; the FC holds the height, this only tells the PC where it is."""
    def __init__(self) -> None:
        self.x = np.zeros(2); self.P = np.diag([0.01, 0.1]); self.t = None

    def predict(self, t: float) -> None:
        if self.t is None: self.t = t; return
        dt = min(max(t - self.t, 0.0), 0.2); self.t = t
        F = np.array([[1.0, dt], [0.0, 1.0]]); self.x = F @ self.x; self.P = F @ self.P @ F.T + np.diag([0.0004 * dt, 0.5 * dt])

    def update_range(self, rng: float, roll: float, pitch: float) -> bool:
        if not (rng > 0 and math.isfinite(rng)): return False
        z = rng * math.cos(roll) * math.cos(pitch); R = (0.02 + 0.01 * rng) ** 2; H = np.array([1.0, 0.0]); S = float(H @ self.P @ H) + R; y = z - self.x[0]
        if y * y / S > NIS_GATE: return False
        K = (self.P @ H) / S; self.x = self.x + K * y; self.P = (np.eye(2) - np.outer(K, H)) @ self.P
        return True
