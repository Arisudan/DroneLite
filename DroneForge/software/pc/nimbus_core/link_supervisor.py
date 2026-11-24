"""LinkSupervisor: offline (no link open) | connecting (open, no frame yet) | connected | degraded | lost from frame age and link quality, with hysteresis. Qt-free; the clock is injectable.
degraded: LQ < 50 or no data for > 0.5 s.  lost: no data for > 2 s, or LQ 0 for 1 s.  Recovery to connected needs 1 s of good data."""
from __future__ import annotations

import time
from typing import Callable, Optional

from . import topics as T

LQ_DEGRADED, AGE_DEGRADED_S, AGE_LOST_S, LQ0_LOST_S, RECOVER_S = 50.0, 0.5, 2.0, 1.0, 1.0


class LinkSupervisor:
    def __init__(self, core, clock: Callable[[], float] = time.monotonic) -> None:
        self.core, self.clock = core, clock
        self._state = "connecting"
        self._since = clock()
        self._last_rx: Optional[float] = None
        self._lq: Optional[float] = None
        self._lq0_since: Optional[float] = None
        self._good_since: Optional[float] = None
        core.bus.subscribe(T.FRAME_IN, self._on_frame)
        core.bus.subscribe(T.TELEMETRY, self._on_tel)

    def _on_frame(self, _f) -> None:
        self._last_rx = self.clock()

    def _on_tel(self, msg) -> None:
        self._last_rx = self.clock()
        name, d = msg
        if name == "link":
            self._lq = d.get("up_lq")

    def state(self) -> str:
        return self._state

    @property
    def since_s(self) -> float:
        return self.clock() - self._since

    def _set(self, s: str, now: float) -> None:
        if s != self._state:
            prev, self._state, self._since = self._state, s, now
            self.core.bus.publish(T.LINK_STATE, {"state": s, "prev": prev, "t": now})

    def update(self, now: Optional[float] = None) -> str:
        now = self.clock() if now is None else now
        if not self.core.connected:
            self._last_rx = self._lq = self._lq0_since = self._good_since = None
            self._set("offline", now)
            return self._state
        if self._last_rx is None:
            self._set("connecting", now)
            return self._state
        age, lq = now - self._last_rx, self._lq
        if lq is not None and lq <= 0:
            self._lq0_since = self._lq0_since if self._lq0_since is not None else now
        else:
            self._lq0_since = None
        lost = age > AGE_LOST_S or (self._lq0_since is not None and now - self._lq0_since >= LQ0_LOST_S)
        degraded = lost or age > AGE_DEGRADED_S or (lq is not None and lq < LQ_DEGRADED)
        if degraded:
            self._good_since = None
            self._set("lost" if lost else ("degraded" if self._state != "lost" else "lost"), now)
        else:
            if self._state in ("connecting",):
                self._set("connected", now)
            elif self._state != "connected":
                self._good_since = self._good_since if self._good_since is not None else now
                if now - self._good_since >= RECOVER_S:
                    self._set("connected", now)
        return self._state
