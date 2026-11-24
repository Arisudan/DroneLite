"""Video health: fps, frame spacing, jitter, freeze detection and the delays in milliseconds. Qt-free; the clock is injectable so tests need no waiting.

What is measured, and what is not:
  * arrival  : every frame the capture thread reads (time.monotonic()).
  * display  : how long after it was read a frame is actually drawn (the GCS's own queue and paint delay).
  * camera -> GCS : ONLY from the latency test (a QR code carrying this PC's own clock is filmed by the drone's camera and read back out of the video), because the camera's clock and this PC's clock
                    are not synchronised, so the stream alone cannot tell the delay. Without the test the line says so.
"""
from __future__ import annotations

import math
import threading
import time
from collections import deque
from typing import Callable, Optional

WINDOW_S = 3.0              # fps / jitter window
FROZEN_S = 1.0              # no new frame for this long: FROZEN
NO_SIGNAL_S = 3.0           # ... for this long: NO SIGNAL
DEGRADED_FPS = 10.0         # below this, or a gap above DEGRADED_GAP_S inside the window: DEGRADED
DEGRADED_GAP_S = 0.25
MIN_FRAMES = 5               # frames since the start before the quality is judged
E2E_KEEP = 20               # latency-test readings kept
E2E_MAX_AGE_S = 60.0


class VideoHealth:
    def __init__(self, clock: Callable[[], float] = time.monotonic) -> None:
        self._clock = clock; self._lock = threading.Lock(); self.reset()

    def reset(self) -> None:
        with self._lock:
            self._frames = deque(); self._shown = deque(); self._e2e = deque()
            self._last = None; self.total = 0

    # ---- feeding
    def on_frame(self, t: Optional[float] = None) -> None:
        t = self._clock() if t is None else t
        with self._lock:
            self._frames.append(t); self._last = t; self.total += 1
            while self._frames and t - self._frames[0] > WINDOW_S + 1.0:
                self._frames.popleft()

    def on_shown(self, t_read: float, t_shown: Optional[float] = None) -> None:
        t_shown = self._clock() if t_shown is None else t_shown
        with self._lock:
            self._shown.append((t_shown, (t_shown - t_read) * 1000.0))
            while self._shown and t_shown - self._shown[0][0] > WINDOW_S:
                self._shown.popleft()

    def on_e2e(self, ms: float, t: Optional[float] = None) -> None:
        t = self._clock() if t is None else t
        with self._lock:
            self._e2e.append((t, float(ms)))
            while len(self._e2e) > E2E_KEEP or (self._e2e and t - self._e2e[0][0] > E2E_MAX_AGE_S):
                self._e2e.popleft()

    # ---- reading
    def snapshot(self, running: bool = True, connecting: bool = False) -> dict:
        now = self._clock()
        with self._lock:
            fr = [t for t in self._frames if now - t <= WINDOW_S]
            shown = [v for t, v in self._shown if now - t <= WINDOW_S]
            e2e = [v for t, v in self._e2e if now - t <= E2E_MAX_AGE_S]
            last = self._last
        iv = [(b - a) * 1000.0 for a, b in zip(fr, fr[1:])]
        fps = (len(fr) - 1) / (fr[-1] - fr[0]) if len(fr) >= 2 and fr[-1] > fr[0] else 0.0
        mean = sum(iv) / len(iv) if iv else 0.0
        jitter = math.sqrt(sum((x - mean) ** 2 for x in iv) / len(iv)) if iv else 0.0
        age = (now - last) if last is not None else float("inf")
        if not running:
            state = "STOPPED"
        elif last is None:
            state = "CONNECTING" if connecting else "NO SIGNAL"
        elif age > NO_SIGNAL_S:
            state = "NO SIGNAL"
        elif age > FROZEN_S:
            state = "FROZEN"
        elif self.total < MIN_FRAMES:
            state = "STARTING"                       # the first few frames: too little to judge, neutral
        elif fps < DEGRADED_FPS or (iv and max(iv) > DEGRADED_GAP_S * 1000.0):
            state = "DEGRADED"
        else:
            state = "LIVE"
        return {"state": state, "fps": fps, "interval_ms": mean, "interval_max_ms": max(iv) if iv else 0.0, "jitter_ms": jitter, "age_s": age,
                "display_ms": (sum(shown) / len(shown)) if shown else None,
                "e2e_ms": e2e[-1] if e2e else None, "e2e_avg_ms": (sum(e2e) / len(e2e)) if e2e else None,
                "e2e_min_ms": min(e2e) if e2e else None, "e2e_max_ms": max(e2e) if e2e else None, "e2e_n": len(e2e)}


def format_line(d: dict, compact: bool = False) -> str:
    """One readable line. The state word is always first, so it reads without colour."""
    st = d["state"]
    if st in ("STOPPED", "CONNECTING", "STARTING"):
        return st
    if st == "NO SIGNAL":
        return "NO SIGNAL" if d["age_s"] == float("inf") else f"NO SIGNAL · last frame {d['age_s']:.0f} s ago"
    if st == "FROZEN":
        return f"FROZEN · no frame for {d['age_s']:.1f} s"
    parts = [st, f"{d['fps']:.0f} fps"]
    if not compact:
        parts.append(f"frame every {d['interval_ms']:.0f} ms (longest {d['interval_max_ms']:.0f}, jitter {d['jitter_ms']:.0f})")
    if d["display_ms"] is not None:
        parts.append(f"drawn {d['display_ms']:.0f} ms after arrival")
    if d["e2e_ms"] is not None:
        parts.append(f"camera to GCS {d['e2e_ms']:.0f} ms (avg {d['e2e_avg_ms']:.0f}, min {d['e2e_min_ms']:.0f}, max {d['e2e_max_ms']:.0f}, {d['e2e_n']} readings)")
        if d["display_ms"] is not None:
            parts.append(f"camera to screen {d['e2e_ms'] + d['display_ms']:.0f} ms")
    elif not compact:
        parts.append("camera delay not measured: press Latency test")
    return " · ".join(parts)
