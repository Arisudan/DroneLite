"""Bench mode (props OFF): the REAL FC and radio link stay in the loop (arming, mixer, motors), the SENSORS are simulated.

The plant is the simulator driven by the FC's real motor outputs (MSP, read-only, on the FC's USB port); its range/flow/attitude replace the real
ones on the bus (the real ones cannot change with props off). Throttle sent to the FC is scaled down (default x0.4) so the motors stay slow; the
plant undoes the scaling. ACRO is selected on the bench so the still IMU cannot wind the FC's I-term up. Same method as nimbus_pc/hover/hil.py.
Collective thrust = mean of the motor outputs, except when a motor sits at the idle floor with a big spread (bench wind-up): then the verified throttle map is used."""
from __future__ import annotations

import time
from typing import Callable, Optional

from nimbus_pc.hover.sim import SimParams

from .simsession import SimSession

SIM_TOPICS = ("optrange", "attitude")


class BenchPlant(SimSession):
    def __init__(self, fc, scale: float = 0.4, idle: float = 0.055, min_check: float = 1050.0, on_telemetry: Optional[Callable] = None, params: Optional[SimParams] = None, **kw) -> None:
        super().__init__(params or SimParams(), 1, on_telemetry=lambda n, d: on_telemetry(n, d) if (on_telemetry and n in SIM_TOPICS) else None, on_frame=None, **kw)
        self.sim.p = self.params = type(self.params)(**{**self.params.__dict__, "ctrl_delay_s": 0.0})   # the real link provides the delay
        self.fc, self.scale, self.idle, self.min_check = fc, scale, idle, min_check
        self.motors = [1000] * 4
        self.fc_armed_real = False
        self.thr_equiv = 0.0
        self.n_fallback = self.n_total = 0
        self._k = 0
        self._hist: list = []

    def step(self) -> None:
        with self._lock:
            c = self._controls
        if self._k % 2 == 0:
            m = self.fc.motors()
            if m: self.motors = m
        if self._k % 10 == 0:
            a = self.fc.armed()
            if a is not None: self.fc_armed_real = a
        self._k += 1
        idle, scale = self.idle, self.scale
        mn = (sum(self.motors) / 4 - 1000) / 1000.0
        rc_us = self.min_check + (2000.0 - self.min_check) * max(mn - idle, 0.0) / (1 - idle)      # undo BF: motor = idle + (1-idle)*(rc-min_check)/(2000-min_check)
        thr_sent = max(rc_us - 1000.0, 0.0) / 1000.0 if mn > idle + 0.002 else 0.0
        thr = thr_sent / scale
        self._hist.append(c.throttle if c.arm else 0.0)
        del self._hist[:-3]
        floor = 1000.0 + idle * 1000.0 + 4
        if self.fc_armed_real and min(self.motors) <= floor and (max(self.motors) - min(self.motors)) > 40:
            thr = self._hist[0]                                   # bench wind-up guard: use the commanded throttle with ~20 ms delay
            self.n_fallback += 1
        self.n_total += 1
        self.thr_equiv = thr
        self.fc_armed = self.fc_armed_real
        self.sim.step(self.dt, c.roll, c.pitch, thr, self.fc_armed_real)
        self._emit_due()
