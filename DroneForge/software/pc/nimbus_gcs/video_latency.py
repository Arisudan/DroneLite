"""Camera-to-GCS delay in milliseconds, measured end to end. Qt-free.

The GCS shows a QR code that carries ITS OWN clock (time.monotonic() in ms). The drone's camera is pointed at that screen, so the code travels camera -> encoder -> radio/Wi-Fi -> decoder -> the
capture thread. When a frame is read, the code in it is decoded and compared with the clock at the moment the frame was read: that difference is the delay of everything between the screen and the
GCS (including the camera's exposure and this screen's refresh, a few tens of ms at most). Both clocks are the same computer's, so no synchronisation is needed.
"""
from __future__ import annotations

import threading
import time
from typing import Callable, Optional

import numpy as np

try:
    import cv2                                              # type: ignore
except Exception:                                           # noqa: BLE001
    cv2 = None

MOD_MS = 10_000_000         # the code carries the clock modulo this (about 2.8 hours)
MAX_PLAUSIBLE_MS = 5000     # a reading above this is a stale or wrong code, dropped
PREFIX = "DF"
MAX_WIDTH = 960             # frames are scaled down to this width before the (slow) QR detector


def clock_ms(t: Optional[float] = None) -> int:
    return int((time.monotonic() if t is None else t) * 1000.0) % MOD_MS


def render_code(ms: int, scale: int = 10, border: int = 3) -> np.ndarray:
    """Black code on white, BGR, ready to draw. `scale` pixels per QR module."""
    if cv2 is None:
        raise RuntimeError("OpenCV not installed")
    enc = cv2.QRCodeEncoder.create()
    img = enc.encode(f"{PREFIX}{int(ms) % MOD_MS}")
    img = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
    pad = border * scale
    img = cv2.copyMakeBorder(img, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=255)
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR) if img.ndim == 2 else img


def decode_code(frame: np.ndarray) -> Optional[int]:
    """The clock value (ms) carried by a QR code anywhere in `frame`, or None."""
    if cv2 is None or frame is None:
        return None
    h, w = frame.shape[:2]
    if w > MAX_WIDTH:
        frame = cv2.resize(frame, (MAX_WIDTH, int(h * MAX_WIDTH / w)), interpolation=cv2.INTER_AREA)
    try:
        val, _pts, _st = cv2.QRCodeDetector().detectAndDecode(frame)
    except Exception:                                       # noqa: BLE001
        return None
    if not val or not val.startswith(PREFIX):
        return None
    try:
        return int(val[len(PREFIX):])
    except ValueError:
        return None


def delay_ms(sent_ms: int, t_read: float) -> Optional[float]:
    """Delay of a frame that was read at `t_read` and carries the code rendered at `sent_ms`; None if implausible."""
    d = (clock_ms(t_read) - int(sent_ms)) % MOD_MS
    return float(d) if 0 <= d <= MAX_PLAUSIBLE_MS else None


class LatencyProbe:
    """Reads the code out of the newest frames in a worker thread (the detector takes tens of ms) and reports the delay."""

    def __init__(self, on_reading: Callable[[float], None], decoder=decode_code) -> None:
        self._on = on_reading; self._decode = decoder
        self._lock = threading.Lock(); self._ev = threading.Event(); self._stop = threading.Event()
        self._pending = None; self._thread = None
        self.attempts = 0; self.found = 0; self.last_found_t = 0.0

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear(); self.attempts = self.found = 0; self.last_found_t = 0.0
        self._thread = threading.Thread(target=self._run, daemon=True); self._thread.start()

    def stop(self) -> None:
        self._stop.set(); self._ev.set()
        t = self._thread
        if t is not None and t is not threading.current_thread():
            t.join(timeout=2)
        self._thread = None

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def feed(self, frame, t_read: float) -> None:
        """Hand over the newest frame (older unread ones are dropped: the probe never falls behind)."""
        with self._lock:
            self._pending = (frame, t_read)
        self._ev.set()

    def _run(self) -> None:
        while not self._stop.is_set():
            self._ev.wait(0.5); self._ev.clear()
            with self._lock:
                job, self._pending = self._pending, None
            if job is None or self._stop.is_set():
                continue
            frame, t_read = job; self.attempts += 1
            sent = self._decode(frame)
            if sent is None:
                continue
            d = delay_ms(sent, t_read)
            if d is not None:
                self.found += 1; self.last_found_t = time.monotonic(); self._on(d)
