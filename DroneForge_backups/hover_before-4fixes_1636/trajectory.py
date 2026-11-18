"""Jerk-limited 1-D reference generator (S-curve), pure Python.

Nimbus plans its set-points with speed 0.25 m/s, acceleration 0.35 m/s^2 and jerk 0.8 m/s^3 (see docs/nimbus_analysis/03_control-and-planning.md, section 3.2).
`JerkLimited1D` reproduces that for one axis: `plan(x0, v0, x1)` then `step(dt) -> (pos, vel, acc)`. Velocity, acceleration and jerk never exceed the limits,
a move can be re-planned from any (pos, vel) (reversals included) and ends exactly on x1 with zero velocity and acceleration.

Method: accelerate towards +-v_max with a jerk-limited velocity law; at every step the braking path (same law towards zero velocity) is simulated ahead and as soon as
braking now would stop at or beyond the target the braking path is latched. A residual of a few mm is finished by a second tiny move. The acceleration is slewed at
+-j_max, so every limit holds by construction at every step.
"""
from __future__ import annotations

import math


class JerkLimited1D:
    def __init__(self, v_max: float = 0.25, a_max: float = 0.35, j_max: float = 0.8):
        self.v_max, self.a_max, self.j_max = v_max, a_max, j_max
        self.x = self.v = self.a = 0.0
        self.target = 0.0
        self.done = True
        self.max_step = 0.005                    # larger dt are split into sub-steps of at most this length
        self._brake = False

    def plan(self, x0: float, v0: float, x1: float) -> None:
        self.x, self.v, self.a, self.target = x0, v0, 0.0, x1
        self.done = False
        self._brake = False
        self._check_done()

    def _check_done(self) -> None:
        if abs(self.target - self.x) < 2e-4 and abs(self.v) < 2e-3:
            self.x, self.v, self.a, self.done = self.target, 0.0, 0.0, True

    def step(self, dt: float) -> tuple[float, float, float]:
        n = max(1, int(math.ceil(dt / self.max_step)))
        for _ in range(n):
            if self.done:
                break
            self._sub(dt / n)
        return self.x, self.v, self.a

    def _law(self, v: float, a: float, vd: float, h: float) -> tuple[float, float]:
        """One step of the jerk-limited velocity tracker towards velocity vd: returns (v, a)."""
        dv = vd - v
        a_des = math.copysign(min(self.a_max, math.sqrt(2.0 * self.j_max * abs(dv))), dv)
        a_new = min(max(a + min(max(a_des - a, -self.j_max * h), self.j_max * h), -self.a_max), self.a_max)
        v_new = v + 0.5 * (a + a_new) * h
        if abs(v_new) > self.v_max and abs(v_new) > abs(v):            # never above the speed limit (an initial v0 above it just decays)
            v_new = math.copysign(self.v_max, v_new)
        return v_new, a_new

    def _stop_point(self, x: float, v: float, a: float) -> float:
        h = 0.01
        for _ in range(2000):
            if abs(v) < 2e-4 and abs(a) < 3e-3:
                break
            v2, a2 = self._law(v, a, 0.0, h)
            x += 0.5 * (v + v2) * h
            v, a = v2, a2
        return x

    def _sub(self, h: float) -> None:
        e = self.target - self.x
        s = 1.0 if e >= 0.0 else -1.0
        if not self._brake:
            v1, a1 = self._law(self.v, self.a, s * self.v_max, h)
            x1 = self.x + 0.5 * (self.v + v1) * h
            if (self._stop_point(x1, v1, a1) - self.target) * s > 0.0 and self.v * s > 0.0:     # accelerating one more step would overshoot: brake now
                self._brake = True
        if self._brake:
            v1, a1 = self._law(self.v, self.a, 0.0, h)
            x1 = self.x + 0.5 * (self.v + v1) * h
            if abs(v1) < 2e-4 and abs(a1) < 3e-3:                        # at rest: if a residual is left, a new (tiny) move finishes it
                self.x, self.v, self.a = x1, 0.0, 0.0
                self._brake = False
                self._check_done()
                return
        self.x, self.v, self.a = x1, v1, a1
