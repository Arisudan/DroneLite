"""Shared video source (DJI O4 via CosmoStreamer URL, USB capture, none). Qt-light: no widgets here.

One instance lives on `win.video_source` (see `get_source`), used by VideoPage and VideoPip.
A capture thread (cv2.VideoCapture, any URL OpenCV/FFmpeg opens: rtsp://, http://.../mjpeg, udp://, a file)
keeps the latest BGR frame. Auto-reconnect with backoff. Tests inject an `opener` so no camera/network is needed.
"""
from __future__ import annotations

import json
import os
import threading
import time

try:                                                        # optional dependency
    import cv2                                              # type: ignore
except Exception:                                           # noqa: BLE001
    cv2 = None

from .video_health import VideoHealth

KINDS = ("network", "usb", "none")
NO_CV2 = "OpenCV not installed (pip install opencv-python)"


def state_path() -> str:
    return os.environ.get("DRONEFORGE_VIDEO_STATE") or os.path.expanduser("~/.config/droneforge/video.json")


def load_choice() -> dict:
    try:
        with open(state_path()) as f:
            d = json.load(f)
        kind = d.get("kind", "none")
        return {"kind": kind if kind in KINDS else "none", "target": str(d.get("target", "")),
                "analog": bool(d.get("analog", False))}
    except Exception:                                       # noqa: BLE001
        return {"kind": "none", "target": "", "analog": False}


def save_choice(kind: str, target: str, analog: bool = False) -> None:
    try:
        p = state_path(); os.makedirs(os.path.dirname(p), exist_ok=True)
        with open(p, "w") as f:
            json.dump({"kind": kind, "target": target, "analog": analog}, f)
    except Exception:                                       # noqa: BLE001
        pass


class FakeCapture:
    """Test helper: wraps `fn() -> ndarray | None` (None = read failure) as a cv2-like capture."""
    def __init__(self, fn, opened: bool = True) -> None:
        self.fn = fn; self.opened = opened

    def isOpened(self) -> bool:
        return self.opened

    def read(self):
        f = self.fn()
        return (f is not None, f)

    def release(self) -> None:
        self.opened = False


def decoder_threads() -> int:
    """Decoder threads for network video. OpenCV's default (several threads) makes FFmpeg hold back about one frame per thread: measured 2026-11-18 on a loopback H.264 stream at 30 fps,
    OpenCV defaults 309 ms against 74 ms with ONE thread (the buffering options nobuffer/low_delay changed nothing). DRONEFORGE_VIDEO_THREADS overrides (0 = OpenCV's default)."""
    try:
        return max(0, int(os.environ.get("DRONEFORGE_VIDEO_THREADS", "1")))
    except ValueError:
        return 1


def open_capture(url: str):
    """cv2.VideoCapture over FFmpeg with the low-delay decoder setting (one decoder thread)."""
    n = decoder_threads()
    if n > 0:
        try:
            return cv2.VideoCapture(url, cv2.CAP_FFMPEG, [cv2.CAP_PROP_N_THREADS, n])
        except Exception:                                   # noqa: BLE001  (an OpenCV without the params form)
            pass
    return cv2.VideoCapture(url, cv2.CAP_FFMPEG)


def default_open(kind: str, target: str):
    if cv2 is None:
        raise RuntimeError(NO_CV2)
    if kind == "usb":
        digits = "".join(c for c in target if c.isdigit())
        return cv2.VideoCapture(int(digits)) if digits else cv2.VideoCapture(target)
    from .video_udp import open_udp, parse_udp_target         # `udp://0.0.0.0:3000` / `3000`: raw H.264/H.265 or MPEG-TS, recognised and relayed here
    port = parse_udp_target(target)
    if port is not None:
        return open_udp(port)
    if target.lower().startswith(("rtsp://", "rtsps://")):
        if RTSP_DIRECT:                                         # CosmoStreamer RTSP read directly (rtsp_direct.py: own RTSP client, picture out on its last packet)
            try:
                from .rtsp_direct import RtspDirectCapture
                return RtspDirectCapture(target)
            except Exception:                                   # noqa: BLE001  (no PyAV, or the server wants something this client does not do): FFmpeg's reader
                pass
            from .video_udp import open_rtsp_direct
            cap = open_rtsp_direct(target)
            if cap is not None: return cap
        return open_rtsp(target)                                # no PyAV: OpenCV
    # Network streams: lowest-latency FFmpeg options (no input buffering, no probing, no frame re-ordering delay). Set before the capture opens;
    # an owner-set OPENCV_FFMPEG_CAPTURE_OPTIONS always wins. UDP also gets a larger receive FIFO so bursts are not dropped.
    import os
    os.environ.setdefault("OPENCV_FFMPEG_CAPTURE_OPTIONS", "fflags;nobuffer|flags;low_delay|probesize;32768|analyzeduration;0|max_delay;0|timeout;5000000")
    if target.lower().startswith("udp://") and "fifo_size" not in target:
        target += ("&" if "?" in target else "?") + "fifo_size=1000000&overrun_nonfatal=1"
    cap = open_capture(target)
    try:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    except Exception:
        pass
    return cap


# RTSP (CosmoStreamer's RTSP server, e.g. rtsp://192.168.50.1:554/video): no input buffering, no probing, no re-ordering queue. RTP over TCP first, then UDP if TCP does not open:
# UDP loses packets over Wi-Fi and FFmpeg cannot recover them (smeared frames; the Flop project measured this and moved to TCP, which costs a little latency). DRONEFORGE_RTSP_TRANSPORT=udp
# (or tcp) chooses the first one to try. An owner-set OPENCV_FFMPEG_CAPTURE_OPTIONS (set before this program started) always wins.
_OWNER_FFMPEG_OPTIONS = os.environ.get("OPENCV_FFMPEG_CAPTURE_OPTIONS")
RTSP_COMMON = "fflags;nobuffer|flags;low_delay|probesize;32768|analyzeduration;0|max_delay;0|reorder_queue_size;0|timeout;5000000"
def rtsp_transports() -> tuple:
    first = (os.environ.get("DRONEFORGE_RTSP_TRANSPORT") or "tcp").strip().lower()
    first = first if first in ("tcp", "udp") else "tcp"
    return (first, "udp" if first == "tcp" else "tcp")


RTSP_DIRECT = True             # False: the older OpenCV reader
RTSP_TRANSPORTS = None          # None = rtsp_transports() at open time


def rtsp_options(transport: str) -> str:
    return f"rtsp_transport;{transport}|{RTSP_COMMON}"


def open_rtsp(url: str, opener=None, transports=None):
    """Open an rtsp:// URL; returns the capture that opened (or the last one that did not). `opener(url)` is injectable for tests.
    The FFmpeg option string is global to the process: it is set only for the duration of each open and then put back."""
    opener = opener or open_capture; transports = transports or rtsp_transports()
    before = os.environ.get("OPENCV_FFMPEG_CAPTURE_OPTIONS"); cap = None
    try:
        for tr in transports:
            os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = _OWNER_FFMPEG_OPTIONS or rtsp_options(tr)
            cap = opener(url)
            if cap is not None and cap.isOpened():
                try:
                    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                except Exception:                           # noqa: BLE001
                    pass
                return cap
            try:
                if cap is not None:
                    cap.release()
            except Exception:                               # noqa: BLE001
                pass
            if _OWNER_FFMPEG_OPTIONS:
                break
        return cap
    finally:
        if before is None:
            os.environ.pop("OPENCV_FFMPEG_CAPTURE_OPTIONS", None)
        else:
            os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = before


class VideoSource:
    def __init__(self, opener=None, stall_s: float = 3.0, backoff=(0.05, 0.3, 1.0, 2.0), load: bool = True) -> None:
        self._opener = opener or default_open
        self.stall_s = stall_s; self.backoff = tuple(backoff)
        self.kind = "none"; self.target = ""; self.analog = False
        if load:
            c = load_choice(); self.kind, self.target, self.analog = c["kind"], c["target"], c["analog"]
        self._lock = threading.Lock()
        self._frame = None; self._frame_t = 0.0; self.frame_count = 0; self._last_t = 0.0; self._t0 = 0.0; self._fps_n = 0; self.fps = 0.0
        self.health = VideoHealth()                          # fps, jitter, freeze and the delays in ms (video_health.py)
        self._state = "stopped"; self.running = False
        self._thread = None; self._stop = threading.Event()
        if opener is None and cv2 is None:
            self._state = NO_CV2

    # ---- config
    def set_source(self, kind: str, target: str = "", analog: bool | None = None) -> None:
        if kind not in KINDS:
            kind = "none"
        if kind == self.kind and target.strip() == self.target and (analog is None or analog == self.analog):
            return                                          # already this source: no restart (2026-11-24: a needless restart froze the GUI for 3 s on the first VIDEO tab)
        was = self.running
        self.stop()
        self.kind, self.target = kind, target.strip()
        if analog is not None:
            self.analog = analog
        save_choice(self.kind, self.target, self.analog)
        if was and kind != "none":
            self.start()

    # ---- state
    @property
    def state(self) -> str:
        with self._lock:
            s = self._state
        if s == "live" and self.frame_age() > self.stall_s:
            return "stalled"
        return s

    HOLD_S = 2.0          # a picture is kept on screen this long through a drop / reconnect (with a tag), so a Wi-Fi hiccup does not blank the view

    def held_frame(self):
        """(frame, t) of the last picture while it is younger than HOLD_S, whatever the state (reconnecting included); else (None, 0.0)."""
        with self._lock:
            f, t = self._frame, self._frame_t
        return (f, t) if (f is not None and self.frame_age() <= self.HOLD_S) else (None, 0.0)

    def frame_age(self) -> float:
        return (time.monotonic() - self._last_t) if self._last_t else float("inf")

    def latest(self):
        with self._lock:
            return self._frame

    def latest_with_time(self):
        """(frame, time.monotonic() when the capture thread read it): the time is what the display delay is measured from."""
        with self._lock:
            return self._frame, self._frame_t

    def note_shown(self, t_read: float) -> None:
        """A display calls this when it has drawn the frame that was read at t_read."""
        self.health.on_shown(t_read)

    def health_snapshot(self) -> dict:
        return self.health.snapshot(running=self.running, connecting=self._state == "connecting")

    def _set(self, s: str) -> None:
        with self._lock:
            self._state = s

    # ---- run
    def start(self) -> None:
        if self.running:
            return
        if self.kind == "none" or (self.kind == "network" and not self.target):
            self._set("error: no source set" if self.kind != "none" else "stopped"); return
        if self._opener is default_open and cv2 is None:
            self._set(NO_CV2); return
        self._stop = threading.Event(); self.running = True; self._set("connecting")      # a NEW stop flag per run: a thread still stuck in an old connect can never write into this run
        self._thread = threading.Thread(target=self._run, args=(self._stop, self.kind, self.target), daemon=True); self._thread.start()

    def stop(self) -> None:
        self._stop.set(); self.running = False
        t = self._thread
        if t is not None and t is not threading.current_thread():
            t.join(timeout=0.15)                            # never block the GUI: a thread stuck in a network connect (up to 5 s) ends by itself in the background
        self._thread = None
        with self._lock:
            self._frame = None
            if not self._state.startswith("OpenCV"):
                self._state = "stopped"
        self._last_t = 0.0; self.fps = 0.0; self.health.reset()

    def _run(self, stop: threading.Event = None, kind: str = None, target: str = None) -> None:
        stop = stop or self._stop; kind = kind or self.kind; target = self.target if target is None else target
        attempt = 0

        def put(st: str) -> None:                           # a stopped (old) run never changes the state the GUI shows
            with self._lock:
                if not stop.is_set(): self._state = st
        while not stop.is_set():
            put("connecting"); cap = None
            try:
                cap = self._opener(kind, target)
                if stop.is_set():
                    break                                   # stopped while connecting: this run is over
                if cap is None or not cap.isOpened():
                    raise RuntimeError("cannot open " + (target or "device"))
                attempt = 0; self._t0 = time.monotonic(); self._fps_n = 0
                while not stop.is_set():
                    ok, frame = cap.read()
                    if not ok or frame is None:
                        raise RuntimeError("lost frames")
                    now = time.monotonic()
                    with self._lock:
                        if stop.is_set():
                            break
                        self._frame = frame; self._frame_t = now; self.frame_count += 1; self._state = "live"
                    self._last_t = now; self._fps_n += 1; self.health.on_frame(now)
                    if now - self._t0 >= 1.0:
                        self.fps = self._fps_n / (now - self._t0); self._t0 = now; self._fps_n = 0
                    time.sleep(0.001)
            except Exception as e:                          # noqa: BLE001
                if stop.is_set():
                    break
                put(f"error: {e}")
            finally:
                try:
                    if cap is not None:
                        cap.release()
                except Exception:                           # noqa: BLE001
                    pass
            wait = self.backoff[min(attempt, len(self.backoff) - 1)]; attempt += 1
            stop.wait(wait)

    # ---- snapshot
    def snapshot_png(self, path: str) -> bool:
        f = self.latest()
        if f is None:
            return False
        try:
            os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
            if cv2 is not None:
                return bool(cv2.imwrite(path, f))
            from PyQt6.QtGui import QImage
            h, w = f.shape[:2]
            return QImage(f.tobytes(), w, h, 3 * w, QImage.Format.Format_BGR888).save(path, "PNG")
        except Exception:                                   # noqa: BLE001
            return False


def get_source(win) -> VideoSource:
    src = getattr(win, "video_source", None)
    if src is None:
        src = VideoSource(); win.video_source = src
    return src
