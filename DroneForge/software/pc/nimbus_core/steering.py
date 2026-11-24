"""Steering input for EXPLORE: the PC autonomy (explorer / roamer, a separate process) sends its stick wishes here and the GCS puts them on the radio.

The same path on the simulation and on the real drone: explorer -> UDP 127.0.0.1:5766 (JSON {"roll", "pitch", "yaw", "seq", "done"?}) -> this listener -> HoverAutonomy.controls() -> CRSF.
Betaflight stick conventions, -1..1: roll + = right, pitch + = forward, yaw + = clockwise (right). The GCS only uses them while the operator has EXPLORE on and the FC holds the height;
take-off, land, SAFE and the height stay with the GCS. Stale input (no packet for STALE_S) = centred sticks, so the FC's own position hold takes over. Localhost only."""
from __future__ import annotations

import json
import socket
import threading
import time
from typing import Optional

PORT = 5766
STALE_S = 0.3              # older than this: centred sticks (the FC holds the position)
MAX_TILT_STICK = 0.40      # roll / pitch cap: 0.40 = 10.9 deg through the Nimbus angle curve
MAX_YAW_STICK = 0.50


def _clip(v, lim: float) -> float:
    try:
        v = float(v)
    except (TypeError, ValueError):
        return 0.0
    if v != v:                # NaN
        return 0.0
    return max(-lim, min(lim, v))


class SteerLink:
    """Listens on 127.0.0.1:PORT in a thread. latest(now) -> (roll, pitch, yaw) or None when stale/absent."""

    def __init__(self, port: int = PORT, clock=time.monotonic) -> None:
        self.port = port; self.clock = clock
        self._lock = threading.Lock(); self._last = None; self._t = None; self.done = False; self.packets = 0; self._times: list = []
        self._sock: Optional[socket.socket] = None; self._thread: Optional[threading.Thread] = None; self._stop = threading.Event()
        self.error = ""
        self.done_reason = ""
        self.active = False          # set by the autonomy: True while EXPLORE applies the sticks; every packet is answered with {"explore": active} so the explorer pauses when it is off

    def start(self) -> bool:
        if self._thread is not None:
            return True
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.bind(("127.0.0.1", self.port)); s.settimeout(0.2)
        except OSError as e:
            self.error = f"steering port {self.port} unavailable: {e.strerror or e}"; return False
        self._sock = s; self._stop.clear(); self.error = ""
        self._thread = threading.Thread(target=self._run, name="steer-link", daemon=True); self._thread.start()
        return True

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._sock is not None:
            self._sock.close()
        self._sock = self._thread = None
        with self._lock:
            self._last = self._t = None; self.done = False

    @property
    def running(self) -> bool:
        return self._thread is not None

    def feed(self, data: bytes, now: Optional[float] = None) -> bool:
        """One packet (also used by tests). Returns False for a malformed one."""
        try:
            d = json.loads(data.decode("utf-8", "replace"))
            if not isinstance(d, dict):
                return False
        except ValueError:
            return False
        now = self.clock() if now is None else now
        with self._lock:
            if d.get("done"):
                self.done = True; self.done_reason = str(d.get("reason", ""))[:80]; self._last = None; self._t = None; return True
            self._last = (_clip(d.get("roll", 0.0), MAX_TILT_STICK), _clip(d.get("pitch", 0.0), MAX_TILT_STICK), _clip(d.get("yaw", 0.0), MAX_YAW_STICK))
            self._t = now; self.packets += 1; self._times = [t for t in self._times if now - t < 1.0] + [now]
        return True

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                data, addr = self._sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            self.feed(data)
            try:
                self._sock.sendto(json.dumps({"explore": bool(self.active)}).encode(), addr)
            except OSError:
                pass

    def latest(self, now: Optional[float] = None):
        now = self.clock() if now is None else now
        with self._lock:
            if self._last is None or self._t is None or now - self._t > STALE_S:
                return None
            return self._last

    def rate_hz(self, now: Optional[float] = None) -> float:
        now = self.clock() if now is None else now
        with self._lock:
            return float(len([t for t in self._times if now - t < 1.0]))
