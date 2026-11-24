"""The GCS feeds the mapping pipeline (realmap/o4_map_node.py: DA3 depth + FC odometry -> RTAB-Map -> RViz) with what only it has: the decoded O4 video and the FC telemetry.

  frames     -> shared memory "droneforge_o4_frames": a 64-byte header (seq u64 [odd while writing], height u32, width u32, capture time f64; byte 32: f64 the reader's
                last read, wall time) + BGR pixels. Frames are copied ONLY while a reader (o4_map_node) has read in the last 3 s: no map running = no copying.
  telemetry  -> UDP JSON to 127.0.0.1:5767 on every FC state update (range m, flow speed forward/right m/s, flow quality, attitude rad as the FC reports it, armed)

Cheap (one 0.9 MB copy per frame); nothing is sent anywhere else."""
from __future__ import annotations

import json
import socket
import struct
import threading
import time

import numpy as np

SHM_NAME = "droneforge_o4_frames"
MAX_W, MAX_H = 1920, 1088
HDR = 64
READER_HB = 32                                                        # o4_map_node writes time.time() here on every read
READER_FRESH_S = 3.0
TELEM_ADDR = ("127.0.0.1", 5767)


class FrameShare:
    def __init__(self, get_frame) -> None:
        from multiprocessing import shared_memory
        try:                                                              # a stale segment from a crashed GCS
            old = shared_memory.SharedMemory(name=SHM_NAME); old.close(); old.unlink()
        except FileNotFoundError:
            pass
        self.shm = shared_memory.SharedMemory(name=SHM_NAME, create=True, size=HDR + MAX_W * MAX_H * 3)
        self.buf = self.shm.buf; self.get_frame = get_frame; self.seq = 0; self._stop = threading.Event(); self._last_t = 0.0; self.frames = 0
        threading.Thread(target=self._run, name="map-frame-share", daemon=True).start()

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                fr = self.get_frame(); f, t = fr if isinstance(fr, tuple) else (fr, time.time())
            except Exception:                                             # noqa: BLE001
                f, t = None, 0.0
            if not self.reader_alive():                                   # no map pipeline reading: copy nothing (item 8, 2026-11-23)
                time.sleep(0.05); continue
            if f is None or t == self._last_t or f.ndim != 3 or f.shape[0] > MAX_H or f.shape[1] > MAX_W:
                time.sleep(0.004); continue
            self._last_t = t; h, w = f.shape[:2]
            if t < 1e9: t = time.time() - (time.monotonic() - t)                   # the video source stamps with the monotonic clock: ROS wants wall time (TF and images must match)
            self.seq += 1; struct.pack_into("<Q", self.buf, 0, 2 * self.seq - 1)                     # odd: writing
            np.ndarray((h, w, 3), np.uint8, buffer=self.buf, offset=HDR)[:] = f
            struct.pack_into("<IId", self.buf, 8, h, w, float(t)); struct.pack_into("<Q", self.buf, 0, 2 * self.seq)   # even: complete
            self.frames += 1

    def reader_alive(self) -> bool:
        try:
            return time.time() - struct.unpack_from("<d", self.buf, READER_HB)[0] < READER_FRESH_S
        except Exception:                                                 # noqa: BLE001
            return False

    def close(self) -> None:
        self._stop.set(); time.sleep(0.06)
        try: self.shm.close(); self.shm.unlink()
        except Exception: pass                                            # noqa: BLE001


class TelemetryFeed:
    """Subscribes to the GCS core's telemetry bus; one UDP datagram per FC state update (the 0xD6 frame, ~18 Hz on the real link)."""

    def __init__(self, core) -> None:
        from nimbus_core import topics as T
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); self.opt = {}; self.att = {}; self.armed = False; self.sent = 0
        core.bus.subscribe(T.TELEMETRY, self._on)

    def _on(self, msg) -> None:
        try:
            name, d = msg
        except Exception:                                                 # noqa: BLE001
            return
        if name == "optrange": self.opt = d; return
        if name == "flight_mode": self.armed = not str(d.get("mode", "")).endswith("*"); return
        if name != "kinematic": return
        o = self.opt
        pkt = {"t": time.time(), "range": o.get("distance_m", 0.0), "flow_f": o.get("velocity_forward", 0.0), "flow_r": o.get("velocity_right", 0.0),
               "q": o.get("flow_quality", 0), "roll": d.get("roll_rad", 0.0), "pitch": d.get("pitch_rad", 0.0), "yaw_cw": d.get("yaw_rad", 0.0),
               "vz": d.get("vz_mps", 0.0), "armed": self.armed}
        try:
            self.sock.sendto(json.dumps(pkt).encode(), TELEM_ADDR); self.sent += 1
        except OSError:
            pass
