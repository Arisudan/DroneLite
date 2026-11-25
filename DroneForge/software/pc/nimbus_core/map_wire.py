"""2D RTAB-Map sparse map for the GCS (replica of the Flop project's map path, minus the Pixhawk/PX4 parts).

Flop: RTAB-Map /map -> map_thinning_node (/map_thin, a 1-pixel wall skeleton) -> tcp_map_streamer_node (port 5765, "DMAP" packets) -> GCS.
Here: the same three steps, but Qt-free and ROS-free so they can be tested without a robot:
  WallSkeleton   the thinning (threshold, hysteresis wall lock, small-speck purge, closing, 1-pixel skeleton)
  MapServer      the TCP broadcaster (runs next to RTAB-Map; a ROS node only feeds it grids)
  MapClient      the GCS side (background thread, reconnects, keeps the newest raw grid, skeleton and drone pose)
Wire format (same as Flop, so either side can be swapped): b"DMAP" + u32 header length + JSON header + u32 payload length + zlib(int8 grid bytes).
The header also carries the drone pose in the map frame ("pose": [x_m, y_m, yaw_rad], yaw counter-clockwise from east), which Flop took from the Pixhawk
and which here comes from the TF map->base_link that RTAB-Map itself corrects (so the marker is always consistent with the map, whatever the FC is).
A client may send b"RESET\\n": the server runs its reset callback (RTAB-Map reset + wall-lock clear) and answers b"RSLT" + u32 length + text.
Only reads and draws; nothing here can send a flight command."""
from __future__ import annotations

import json
import math
import socket
import struct
import threading
import time
import zlib
from typing import Callable, Optional

import numpy as np

MAGIC = b"DMAP"
RESULT_MAGIC = b"RSLT"
DEFAULT_PORT = 5765


# ----------------------------------------------------------------------------------------------------------------------------------- thinning
def _zhang_suen(binary: np.ndarray) -> np.ndarray:
    """Zhang-Suen thinning in numpy: a true 1-pixel centre line that keeps the connectivity of walls, identical on every machine."""
    img = (binary > 0).astype(np.uint8); img = np.pad(img, 1)
    while True:
        changed = False
        for step in (0, 1):
            p2, p3, p4, p5 = img[:-2, 1:-1], img[:-2, 2:], img[1:-1, 2:], img[2:, 2:]
            p6, p7, p8, p9 = img[2:, 1:-1], img[2:, :-2], img[1:-1, :-2], img[:-2, :-2]
            c = img[1:-1, 1:-1]; nb = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9
            seq = [p2, p3, p4, p5, p6, p7, p8, p9, p2]
            trans = sum(((seq[i] == 0) & (seq[i + 1] == 1)).astype(np.uint8) for i in range(8))
            if step == 0: cond = (p2 * p4 * p6 == 0) & (p4 * p6 * p8 == 0)
            else: cond = (p2 * p4 * p8 == 0) & (p2 * p6 * p8 == 0)
            kill = (c == 1) & (nb >= 2) & (nb <= 6) & (trans == 1) & cond
            if kill.any():
                img[1:-1, 1:-1][kill] = 0; changed = True
        if not changed:
            return img[1:-1, 1:-1] * 255


def _skeleton(binary: np.ndarray) -> np.ndarray:
    """1-pixel centre line of a 0/255 image: OpenCV-contrib thinning when the build has it, else Zhang-Suen (same algorithm)."""
    import cv2
    if hasattr(cv2, "ximgproc") and hasattr(cv2.ximgproc, "thinning"):
        try:
            return cv2.ximgproc.thinning(binary)
        except Exception:
            pass
    return _zhang_suen(binary)


class WallSkeleton:
    """Flop's map_thinning_node, as a class. update(grid, origin) -> grid where the walls are single cells of value 100 and everything else keeps its
    free/unknown value. A wall cell once seen with confidence >= lock_threshold stays until it has been seen clear for unlock_miss_count updates, so a
    wall does not flicker when a view of it is blocked for a moment."""
    def __init__(self, lock_threshold: int = 80, occupancy_threshold: int = 65, min_wall_area_pixels: int = 20, unlock_miss_count: int = 15) -> None:
        self.lock_threshold = lock_threshold; self.occ = occupancy_threshold; self.min_area = min_wall_area_pixels; self.unlock = unlock_miss_count
        self.reset()

    def reset(self) -> None:
        self.locked = None; self.miss = None; self.shape = None; self.origin = None

    def update(self, grid: np.ndarray, origin: tuple[float, float]) -> np.ndarray:
        import cv2
        h, w = grid.shape; raw = grid.astype(np.int8)
        if self.shape != (h, w) or (self.origin is not None and math.hypot(origin[0] - self.origin[0], origin[1] - self.origin[1]) > 0.05):
            self.shape = (h, w); self.origin = (float(origin[0]), float(origin[1]))
            self.locked = np.zeros((h, w), np.uint8); self.miss = np.zeros((h, w), np.int32)
        hits = raw >= self.lock_threshold
        self.locked[hits] = 255; self.miss[hits] = 0
        self.miss[(self.locked == 255) & (raw < 40) & (raw >= 0)] += 1
        gone = self.miss >= self.unlock
        self.locked[gone] = 0; self.miss[gone] = 0
        binary = cv2.bitwise_or((raw >= self.occ).astype(np.uint8) * 255, self.locked)
        out = raw.copy(); out[raw >= self.occ] = 0
        if np.count_nonzero(binary) == 0:
            return out
        n, labels, stats, _ = cv2.connectedComponentsWithStats(binary, connectivity=8)
        keep = np.zeros(n, bool); keep[1:] = stats[1:, cv2.CC_STAT_AREA] >= self.min_area
        binary = np.where(keep[labels], 255, 0).astype(np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
        out[_skeleton(binary) > 0] = 100
        return out


# ----------------------------------------------------------------------------------------------------------------------------------- packets
def pack(source: str, grid: np.ndarray, resolution: float, origin: tuple[float, float], seq: int = 0, pose: Optional[tuple[float, float, float]] = None) -> bytes:
    h, w = grid.shape
    hdr = {"seq": seq, "source": source, "width": int(w), "height": int(h), "resolution": float(resolution),
           "origin_x": float(origin[0]), "origin_y": float(origin[1]), "timestamp": time.time(), "pose": list(pose) if pose else None}
    hb = json.dumps(hdr).encode(); body = zlib.compress(np.ascontiguousarray(grid, dtype=np.int8).tobytes(), 1)
    return MAGIC + struct.pack("!I", len(hb)) + hb + struct.pack("!I", len(body)) + body


def _recv_exact(sock: socket.socket, n: int) -> Optional[bytes]:
    buf = bytearray()
    while len(buf) < n:
        try:
            chunk = sock.recv(n - len(buf))
        except socket.timeout:
            raise
        except OSError:
            return None
        if not chunk:
            return None
        buf += chunk
    return bytes(buf)


# ----------------------------------------------------------------------------------------------------------------------------------- server
class MapServer:
    """Broadcasts packets to every connected GCS and sends the newest raw + skeleton packet to a GCS the moment it connects."""
    def __init__(self, port: int = DEFAULT_PORT, host: str = "0.0.0.0", on_reset: Optional[Callable[[], str]] = None) -> None:
        self.port = port; self.host = host; self.on_reset = on_reset
        self._clients: list[socket.socket] = []; self._lock = threading.Lock(); self._latest: dict[str, bytes] = {}; self._running = False
        self._srv: Optional[socket.socket] = None; self._seq = 0

    def start(self) -> None:
        self._srv = socket.socket(); self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1); self._srv.bind((self.host, self.port)); self._srv.listen(5)
        self._srv.settimeout(0.5); self.port = self._srv.getsockname()[1]; self._running = True
        threading.Thread(target=self._accept, daemon=True).start()

    def stop(self) -> None:
        self._running = False
        with self._lock:
            for c in self._clients:
                try: c.close()
                except OSError: pass
            self._clients.clear()
        if self._srv:
            try: self._srv.close()
            except OSError: pass

    def _accept(self) -> None:
        while self._running:
            try:
                c, _ = self._srv.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            c.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            with self._lock:
                self._clients.append(c)
                for pkt in self._latest.values():
                    try: c.settimeout(2.0); c.sendall(pkt)
                    except OSError: pass
            threading.Thread(target=self._commands, args=(c,), daemon=True).start()

    def _commands(self, c: socket.socket) -> None:
        c.settimeout(0.5)
        while self._running:
            try:
                data = c.recv(64)
            except socket.timeout:
                continue
            except OSError:
                break
            if not data:
                break
            if b"RESET" in data.upper():
                try:
                    text = self.on_reset() if self.on_reset else "no reset hook"
                except Exception as e:                                                  # a failed reset must never kill the server
                    text = f"ERR: {e}"
                with self._lock:
                    self._latest.clear()
                    try: c.sendall(RESULT_MAGIC + struct.pack("!I", len(text.encode())) + text.encode())
                    except OSError: pass
        with self._lock:
            if c in self._clients: self._clients.remove(c)
        try: c.close()
        except OSError: pass

    def publish(self, source: str, grid: np.ndarray, resolution: float, origin: tuple[float, float], pose=None) -> None:
        self._seq += 1; pkt = pack(source, grid, resolution, origin, self._seq, pose)
        with self._lock:
            self._latest[source] = pkt
            for c in list(self._clients):
                try: c.settimeout(2.0); c.sendall(pkt)      # 2 s (was 0.5 s: a busy GCS was dropped mid-packet and kept reconnecting)
                except OSError:
                    self._clients.remove(c)
                    try: c.close()
                    except OSError: pass


# ----------------------------------------------------------------------------------------------------------------------------------- client
class MapClient:
    """GCS side. Keeps the newest raw grid ("map_raw"), skeleton ("map_thin") and pose; reconnects by itself; reset() asks the server to restart the map."""
    def __init__(self) -> None:
        self.host = ""; self.port = DEFAULT_PORT; self.running = False; self.raw = None; self.thin = None; self.pose = None; self.t_raw = 0.0; self.t_thin = 0.0
        self.state = "off"; self.reply = ""; self._sock: Optional[socket.socket] = None; self._gen = 0; self._lock = threading.Lock(); self._wake = threading.Event()

    def start(self, host: str, port: int = DEFAULT_PORT) -> None:
        self.stop(); self.host = host; self.port = port; self.running = True; self._gen += 1; self.state = "connecting"; self._wake.clear()
        self._thread = threading.Thread(target=self._loop, args=(self._gen,), daemon=True); self._thread.start()

    def stop(self) -> None:
        self.running = False; self._gen += 1; self.state = "off"; self._wake.set()          # wakes the reconnect wait at once
        s = self._sock
        if s is not None:
            try: s.shutdown(socket.SHUT_RDWR)                       # wakes a recv() in the thread (close() alone does not): a thread still inside recv at interpreter exit crashed the process
            except OSError: pass
            try: s.close()
            except OSError: pass
        th = getattr(self, "_thread", None)
        if th is not None and th is not threading.current_thread(): th.join(timeout=0.5)

    def clear(self) -> None:
        with self._lock:
            self.raw = self.thin = self.pose = None; self.t_raw = self.t_thin = 0.0

    def reset(self) -> bool:
        s = self._sock
        if s is None:
            return False
        try: s.sendall(b"RESET\n"); self.clear(); return True
        except OSError: return False

    def age_s(self) -> float:
        return time.time() - max(self.t_raw, self.t_thin) if (self.t_raw or self.t_thin) else float("inf")

    def _loop(self, gen: int) -> None:
        while self.running and gen == self._gen:
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM); s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 << 20)   # 8 MB: whole map packets fit in the kernel even when this process is busy
                s.settimeout(3.0); s.connect((self.host, self.port)); self._sock = s; s.settimeout(2.0); self.state = "connected"
                while self.running and gen == self._gen:
                    try:
                        magic = _recv_exact(s, 4)
                    except socket.timeout:
                        continue
                    if magic is None:
                        break
                    if magic == RESULT_MAGIC:
                        n = struct.unpack("!I", _recv_exact(s, 4) or b"\0\0\0\0")[0]; self.reply = (_recv_exact(s, n) or b"").decode(errors="replace"); continue
                    if magic != MAGIC:
                        break
                    hl = _recv_exact(s, 4); hdr = _recv_exact(s, struct.unpack("!I", hl)[0]) if hl else None
                    pl = _recv_exact(s, 4) if hdr else None; body = _recv_exact(s, struct.unpack("!I", pl)[0]) if pl else None
                    if body is None:
                        break
                    h = json.loads(hdr); w, ht = h["width"], h["height"]
                    grid = np.frombuffer(zlib.decompress(body), dtype=np.int8)
                    if grid.size != w * ht:
                        continue
                    item = {"grid": grid.reshape(ht, w), "res": h["resolution"], "ox": h["origin_x"], "oy": h["origin_y"], "t": time.time()}
                    with self._lock:
                        if h.get("pose"): self.pose = tuple(h["pose"])
                        if "thin" in h["source"]: self.thin = item; self.t_thin = item["t"]
                        else: self.raw = item; self.t_raw = item["t"]
            except Exception:
                pass
            finally:
                s = self._sock; self._sock = None
                if s is not None:
                    try: s.close()
                    except OSError: pass
            if self.running and gen == self._gen:
                self.state = "connecting"; self._wake.wait(1.0)
