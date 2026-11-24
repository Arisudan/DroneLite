"""Attitude from whichever source is fresher: the ATTITUDE frame (0x1E, ~1.2 Hz since the FC telemetry trim, age median ~0.4 s) or the roll/pitch/yaw inside the D4 EKF-state frame
(~7.5 Hz, age median ~0.05 s). A valid, recent D4 frame is republished as an ordinary "attitude" telemetry message (same keys, radians as the decoder gives them, plus
source="d4"), so the controller, the safety monitor, the director/calibration, the recorder and the GCS all use it without any change. The ATTITUDE frame passes through unchanged
and is the fallback when D4 is stale (> stale_s), invalid or has sentinel values."""
from __future__ import annotations

import math
import time
from typing import Callable, Optional

SENTINEL_RAD = 3.2766            # 0x7FFF / 0x8000 / 10000 = 3.2767 / -3.2768: not initialised (NaN/inf in the FC); also beyond pi
STALE_S = 0.5


def usable_d4_attitude(d: dict) -> Optional[dict]:
    """The attitude dict of a D4 kinematic frame, or None when the frame is not valid (decoder flag, or sentinel / non-finite / impossible angles)."""
    if not d or not d.get("valid", False):
        return None
    try:
        r, p, y = float(d["roll_rad"]), float(d["pitch_rad"]), float(d["yaw_rad"])
    except (KeyError, TypeError, ValueError):
        return None
    if not all(math.isfinite(v) and abs(v) < SENTINEL_RAD for v in (r, p, y)):
        return None
    return {"roll_rad": r, "pitch_rad": p, "yaw_rad": y, "source": "d4"}


class AttitudeSelector:
    def __init__(self, event: Optional[Callable[[str], None]] = None, stale_s: float = STALE_S):
        self.event, self.stale_s = event, stale_s
        self.t_att: Optional[float] = None        # last ATTITUDE frame
        self.t_d4: Optional[float] = None         # last USABLE D4 frame
        self.source = ""                          # "d4" | "att": what the last published attitude came from

    def feed(self, name: str, d: dict, now: Optional[float] = None) -> list:
        """Telemetry message in -> list of (name, dict) messages to publish (the original one, plus an "attitude" derived from D4 when that is the fresher source)."""
        now = time.monotonic() if now is None else now
        if name == "attitude":
            self.t_att = now
            if self.source != "att" and (self.t_d4 is None or now - self.t_d4 > self.stale_s):
                self._switch("att", "attitude frame (D4 stale or invalid)")
            return [(name, d)]
        if name == "kinematic":
            a = usable_d4_attitude(d)
            if a is not None:
                self.t_d4 = now
                if self.source != "d4":
                    self._switch("d4", "D4 EKF frame (fresher than the attitude frame)")
                return [(name, d), ("attitude", a)]
        return [(name, d)]

    def last_time(self, now: Optional[float] = None) -> Optional[float]:
        """Arrival time of the freshest attitude information (either source)."""
        ts = [t for t in (self.t_att, self.t_d4) if t is not None]
        return max(ts) if ts else None

    def _switch(self, src: str, why: str) -> None:
        self.source = src
        if self.event:
            self.event(f"attitude source: {why}")
