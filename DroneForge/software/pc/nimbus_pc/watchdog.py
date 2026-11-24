"""Sensor watchdog: port of NimbusOS nimbus_core `OptrangeFailsafe::Update` (decompiled, 2.9.0).

Rule set read from the binary:
- Only active while the vehicle is armed/active; leaving that state resets everything.
- Engages once throttle >= 5% (constant 0.05 in .rodata) and records the engage time.
- missing-frames: no new sensor sample for >= 0.5 s (measured from the last sample, or from engage time if none yet).
- payload-flatline: a new sample byte-identical to the previous one, with no change for >= 0.5 s.
- Once tripped it latches (state != 0) and the caller forces the throttle channel to minimum (172)
  until the vehicle is disarmed/deactivated.
Which CRSF frame carries the sensor payload in Nimbus's custom Betaflight is NOT known; the caller feeds samples.
"""
from __future__ import annotations

import math
from typing import Optional

OK, FLATLINE, MISSING = 0, 1, 2
NAMES = {OK: "ok", FLATLINE: "payload-flatline", MISSING: "missing-frames"}


class SensorWatchdog:
    def __init__(self, flatline_s: float = 0.5, missing_s: float = 0.5, throttle_threshold: float = 0.05):
        self.flatline_s, self.missing_s, self.throttle_threshold = flatline_s, missing_s, throttle_threshold
        self.state = OK
        self._pending: Optional[tuple[bytes, float]] = None
        self._reset()

    def _reset(self) -> None:
        self.state = OK
        self.engaged = False
        self._engage_time = 0.0
        self._last_sample_ts: Optional[float] = None
        self._last_payload: Optional[bytes] = None
        self._last_change_ts = 0.0

    @property
    def forcing_zero(self) -> bool:
        return self.state != OK

    def feed(self, payload: bytes, timestamp: float) -> None:
        """Record the newest sensor sample (evaluated on the next update())."""
        self._pending = (bytes(payload), timestamp)

    def update(self, active: bool, throttle01: float, now: float) -> int:
        """Returns 0 normally, or FLATLINE/MISSING once, on the call that trips the failsafe."""
        if not active:
            self._reset()
            self._pending = None
            return OK
        if self.state != OK:
            return OK                      # latched: report the trip only once
        if not self.engaged:
            if throttle01 < self.throttle_threshold or math.isnan(throttle01):
                return OK
            self.engaged = True
            self._engage_time = now
        pending, self._pending = self._pending, None
        if pending and (self._last_sample_ts is None or self._last_sample_ts < pending[1]):
            payload, ts = pending
            self._last_sample_ts = ts
            if self._last_payload is None:
                self._last_payload, self._last_change_ts = payload, ts
            elif payload == self._last_payload:
                if max(0.0, ts - self._last_change_ts) >= self.flatline_s:
                    self.state = FLATLINE
                    return FLATLINE
            else:
                self._last_payload, self._last_change_ts = payload, ts
        ref = self._last_sample_ts if self._last_sample_ts is not None else self._engage_time
        if max(0.0, now - ref) >= self.missing_s:
            self.state = MISSING
            return MISSING
        return OK
