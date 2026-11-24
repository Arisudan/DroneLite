"""UDP video input for the GCS: type in `udp://0.0.0.0:3000` (or just `3000`) and press Start.

CosmoStreamer / DJI O4 send RAW H.264 or H.265 over UDP: no container, no timing. OpenCV cannot open that by itself, so this module
  1. listens on the port for a moment and recognises the stream: MPEG-TS, raw H.264, raw H.265 (RTP is reported as unsupported),
  2. for raw video starts a managed `ffmpeg -c copy` helper (no re-encoding, lowest latency, wall-clock timestamps) that re-wraps it as MPEG-TS on a free
     localhost port, and opens that with OpenCV; the helper is stopped when the capture is released,
  3. for MPEG-TS opens the port directly.
No separate relay terminal is needed. Qt-free; the unit tests inject packets, so no network or ffmpeg is needed for the classifier.
"""
from __future__ import annotations

import os
import re
import shutil
import socket
import subprocess
import threading
import time

import numpy as np
from typing import Iterable, List, Optional, Tuple

try:
    import cv2                                               # type: ignore
except Exception:                                            # noqa: BLE001
    cv2 = None

CAPTURE_OPTIONS = "fflags;nobuffer|flags;low_delay|probesize;32768|analyzeduration;0|max_delay;0|timeout;5000000"


# --------------------------------------------------------------------------------------------- address parsing
def parse_udp_target(target: str) -> Optional[int]:
    """`udp://host:port`, `host:port` or `port` -> the UDP port to listen on (we always bind all interfaces); None if it is not a UDP address."""
    t = target.strip()
    m = re.fullmatch(r"(?:udp://)?(?:[\w.\-\[\]:*]*:)?(\d{2,5})(?:\?.*)?", t, re.I)
    if not m or not (t.lower().startswith("udp://") or re.fullmatch(r"(?:[\d.*]+:)?\d{2,5}", t)):
        return None
    port = int(m.group(1))
    return port if 1 <= port <= 65535 else None


# --------------------------------------------------------------------------------------------- classifier
def _nal_headers(data: bytes) -> Iterable[Tuple[int, int]]:
    """Yield (byte0, byte1) after every Annex-B start code (00 00 01)."""
    i = 0
    while True:
        i = data.find(b"\x00\x00\x01", i)
        if i < 0 or i + 4 >= len(data):
            return
        yield data[i + 3], data[i + 4]
        i += 3


def classify(packets: List[bytes]) -> str:
    """'mpegts' | 'rtp' | 'h264' | 'hevc' | 'unknown' from the first UDP payloads."""
    if not packets:
        return "unknown"
    ts = sum(1 for p in packets if len(p) >= 188 and len(p) % 188 == 0 and all(p[k] == 0x47 for k in range(0, len(p), 188)))
    if ts >= max(1, len(packets) // 2):
        return "mpegts"
    if sum(1 for p in packets if len(p) > 12 and p[0] >> 6 == 2 and p[1] & 0x7F in (96, 97, 98, 99, 100, 101, 102, 103)) >= max(1, len(packets) // 2) \
            and not any(p[:3] == b"\x00\x00\x01" or p[:4] == b"\x00\x00\x00\x01" for p in packets):
        return "rtp"
    blob = b"".join(packets)
    h264 = h265 = 0
    for b0, b1 in _nal_headers(blob):
        if b0 & 0x80 == 0:
            t = b0 & 0x1F
            if t in (7, 8):
                h264 += 3
            elif t in (5, 1, 6, 9):
                h264 += 1
        if b0 & 0x81 == 0 and (b1 & 0x07) == 1:
            t = (b0 >> 1) & 0x3F
            if t in (32, 33, 34):
                h265 += 3
            elif t in (19, 20, 21, 1, 0, 35, 39, 40):
                h265 += 1
    if h264 == h265 == 0:
        return "unknown"
    return "hevc" if h265 > h264 else "h264"


# --------------------------------------------------------------------------------------------- helpers
def local_addresses() -> List[str]:
    try:
        out = subprocess.run(["hostname", "-I"], capture_output=True, text=True, timeout=2).stdout.split()
        return [a for a in out if ":" not in a]
    except Exception:                                        # noqa: BLE001
        return []


def sniff(port: int, seconds: float = 6.0, want: int = 40) -> Tuple[str, int]:
    """Listen on `port`; returns (kind, packets_seen). Stops early once the kind is certain (SPS/PPS or TS sync seen)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        s.bind(("0.0.0.0", port))
    except OSError as e:
        s.close()
        raise RuntimeError(f"UDP port {port} is busy ({e.strerror}): close ffplay / another relay using it") from e
    s.settimeout(0.5); pk: List[bytes] = []; t0 = time.monotonic()
    try:
        while time.monotonic() - t0 < seconds:
            try:
                d, _a = s.recvfrom(65535)
            except socket.timeout:
                continue
            pk.append(d)
            if len(pk) >= 8 and len(pk) % 8 == 0:
                k = classify(pk)
                if k in ("mpegts", "rtp") or (k in ("h264", "hevc") and len(pk) >= 16):
                    return k, len(pk)
            if len(pk) >= want:
                break
    finally:
        s.close()
    return classify(pk), len(pk)


def _free_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0)); p = s.getsockname()[1]; s.close()
    return p


class RelayedCapture:
    """cv2-like capture that also owns the ffmpeg helper process."""

    def __init__(self, cap, proc: Optional[subprocess.Popen]) -> None:
        self.cap, self.proc = cap, proc

    def isOpened(self) -> bool:
        return bool(self.cap is not None and self.cap.isOpened())

    def read(self):
        return self.cap.read()

    def get(self, *a):
        return self.cap.get(*a)

    def set(self, *a):
        return self.cap.set(*a)

    def release(self) -> None:
        try:
            if self.cap is not None:
                self.cap.release()
        finally:
            if self.proc is not None and self.proc.poll() is None:
                self.proc.terminate()
                try:
                    self.proc.wait(2)
                except Exception:                            # noqa: BLE001
                    self.proc.kill()


def _open_cv(url: str):
    import os
    os.environ.setdefault("OPENCV_FFMPEG_CAPTURE_OPTIONS", CAPTURE_OPTIONS)
    from .video_source import open_capture                 # one decoder thread: OpenCV's default adds ~230 ms at 30 fps (see video_source.decoder_threads)
    cap = open_capture(url)
    try:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    except Exception:                                        # noqa: BLE001
        pass
    return cap


DIRECT_DECODE = True       # one low-delay ffmpeg decoder -> raw frames on a pipe (2026-11-21: the relay -> MPEG-TS -> OpenCV path measured 137 ms / 22 of 30 fps; this one ~1 frame)


class AvCapture:
    """cv2-like capture decoding raw H.264/H.265 over UDP IN THIS PROCESS (PyAV): socket -> parser -> decoder inside read(), which the VideoSource calls from its
    own capture thread. No helper process, no pipe, no container: a frame is out as soon as the parser sees where it ends (the next start code; the simulator appends an
    access-unit delimiter after every frame, so it ends at once). Measured 2026-11-21: 3 ms median at 30 fps (the relay path: 137 ms, 22 of 30 fps).
    read() returns the NEWEST frame and drops an older backlog."""

    def __init__(self, port: int, kind: str = "h264") -> None:
        import av                                            # optional dependency (pip install av); open_udp falls back without it
        self._av = av
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
        self.sock.bind(("0.0.0.0", port)); self.sock.settimeout(0.05)      # no SO_REUSEADDR: a second listener must fail, not split the packets with this one
        self.cc = av.CodecContext.create("hevc" if kind in ("hevc", "h265") else "h264", "r"); self.cc.thread_type = "SLICE"
        self.cc.flags |= av.codec.context.Flags.low_delay
        self.size = None; self._open = True; self.proc = None          # no helper process (the relay captures have one)

    def _feed(self, data: bytes, last):
        try:
            for pk in self.cc.parse(data):
                for fr in self.cc.decode(pk):
                    last = fr.to_ndarray(format="bgr24"); self.size = (fr.width, fr.height)
        except self._av.error.FFmpegError:                   # a lost packet: the decoder recovers at the next key frame
            pass
        return last

    def isOpened(self) -> bool:
        return self._open

    def read(self):
        if not self._open:
            return False, None
        t0 = time.time(); last = None
        while time.time() - t0 < 1.0:
            try:
                data = self.sock.recv(65536)
            except socket.timeout:
                if last is not None:
                    return True, last
                continue
            except OSError:
                return False, None
            last = self._feed(data, last)
            if last is not None:                             # drain what is already queued: always hand out the newest picture
                self.sock.setblocking(False)
                try:
                    while True:
                        last = self._feed(self.sock.recv(65536), last)
                except (BlockingIOError, OSError):
                    pass
                self.sock.settimeout(0.05)
                return True, last
        return False, None

    def get(self, prop, *a):
        if cv2 is not None and self.size is not None:
            if prop == cv2.CAP_PROP_FRAME_WIDTH: return float(self.size[0])
            if prop == cv2.CAP_PROP_FRAME_HEIGHT: return float(self.size[1])
        return 0.0

    def set(self, *a):
        return False

    def release(self) -> None:
        self._open = False
        try:
            self.sock.close()
        except OSError:
            pass


class AvRtspCapture:
    """cv2-like capture for CosmoStreamer's RTSP server, decoded IN THIS PROCESS (PyAV): no OpenCV, no helper process. A reader thread pulls every packet and
    decodes it at once (the socket is never left to fill up); read() hands out only the NEWEST picture, converted to BGR once. 2026-11-23: the OpenCV reader
    returned frames one by one, so a burst (fast motion = big frames) built a backlog it never caught up on: lag while moving."""

    OPTS = {"fflags": "nobuffer+noparse+nofillin", "flags": "low_delay", "probesize": "32768", "analyzeduration": "0", "max_delay": "0", "reorder_queue_size": "0"}

    def __init__(self, url: str, transport: str = "tcp") -> None:
        import av
        self._av = av; self.url = url; self.size = None; self.proc = None
        self.c = av.open(url, options=dict(self.OPTS, rtsp_transport=transport), timeout=(5.0, 2.0))
        self.vs = self.c.streams.video[0]; cc = self.vs.codec_context
        cc.thread_type = "SLICE"; cc.flags |= av.codec.context.Flags.low_delay          # frame threads would hold back one frame each
        cc.flags2 |= av.codec.context.Flags2.chunks             # with noparse: a picture is out when its LAST slice is decoded, not when the next picture starts (-33 ms)
        self._cv = threading.Condition(); self._fr = None; self._n = 0; self._given = 0; self._open = True; self._err = None
        threading.Thread(target=self._run, name="rtsp-decode", daemon=True).start()

    def _run(self) -> None:
        try:
            for pk in self.c.demux(self.vs):
                if not self._open: break
                try:
                    for fr in pk.decode():
                        with self._cv: self._fr = fr; self._n += 1; self._cv.notify_all()
                except self._av.error.FFmpegError:              # a damaged packet: the next key frame recovers
                    pass
        except Exception as e:                                 # noqa: BLE001
            self._err = e
        with self._cv: self._open = False; self._cv.notify_all()

    def isOpened(self) -> bool:
        return self._open

    def read(self):
        with self._cv:
            if not self._cv.wait_for(lambda: self._n != self._given or not self._open, timeout=1.0) or self._n == self._given: return False, None
            fr = self._fr; self._given = self._n
        self.size = (fr.width, fr.height)
        return True, fr.to_ndarray(format="bgr24")

    def get(self, prop, *a):
        if cv2 is not None and self.size is not None:
            if prop == cv2.CAP_PROP_FRAME_WIDTH: return float(self.size[0])
            if prop == cv2.CAP_PROP_FRAME_HEIGHT: return float(self.size[1])
        return 0.0

    def set(self, *a):
        return False

    def release(self) -> None:
        self._open = False
        try: self.c.close()
        except Exception: pass                                 # noqa: BLE001


def open_rtsp_direct(url: str):
    """The RTSP stream straight from CosmoStreamer (TCP first: UDP loses packets over Wi-Fi; DRONEFORGE_RTSP_TRANSPORT=udp tries UDP first). None when PyAV is missing."""
    try:
        import av  # noqa: F401
    except Exception:                                          # noqa: BLE001
        return None
    first = (os.environ.get("DRONEFORGE_RTSP_TRANSPORT") or "tcp").strip().lower(); order = ("udp", "tcp") if first == "udp" else ("tcp", "udp")
    err = None
    for tr in order:
        try:
            return AvRtspCapture(url, tr)
        except Exception as e:                                 # noqa: BLE001
            err = e
    raise RuntimeError(f"cannot open {url} ({err})")


class PipeCapture:
    """cv2-like capture: ONE ffmpeg process decodes the UDP stream (raw H.264/H.265 or MPEG-TS) with the low-delay flags and writes BGR frames to a pipe;
    a reader thread keeps only the newest frame, so read() never returns a stale backlog. The frame size is taken from ffmpeg's own stream report."""

    _SIZE = re.compile(r"Stream #0:\d+.*Video: rawvideo.*?, (\d{2,5})x(\d{2,5})")

    def __init__(self, kind: str, port: int, ff: str, popen=subprocess.Popen, wait_s: float = 6.0) -> None:
        import threading
        cmd = [ff, "-hide_banner", "-loglevel", "info", "-nostats", "-fflags", "nobuffer", "-flags", "low_delay", "-probesize", "32768", "-analyzeduration", "0",
               "-f", kind, "-i", f"udp://0.0.0.0:{port}?fifo_size=1000000&overrun_nonfatal=1", "-an", "-sn", "-f", "rawvideo", "-pix_fmt", "bgr24", "-flush_packets", "1", "pipe:1"]
        self.proc = popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0)
        self.size = None; self._frame = None; self._n = 0; self._given = 0; self._cv = threading.Condition(); self._t_last = 0.0
        threading.Thread(target=self._err, daemon=True).start()
        t0 = time.time()
        while self.size is None and self.proc.poll() is None and time.time() - t0 < wait_s:
            time.sleep(0.02)
        if self.size is not None:
            threading.Thread(target=self._rd, daemon=True).start()

    def _err(self) -> None:
        for line in iter(self.proc.stderr.readline, b""):
            if self.size is None:
                m = self._SIZE.search(line.decode("utf-8", "replace"))
                if m:
                    self.size = (int(m.group(1)), int(m.group(2)))

    def _rd(self) -> None:
        w, h = self.size; n = w * h * 3; out = self.proc.stdout
        while True:
            buf = bytearray()
            while len(buf) < n:
                b = out.read(n - len(buf))
                if not b:
                    return
                buf += b
            f = np.frombuffer(bytes(buf), np.uint8).reshape(h, w, 3)
            with self._cv:
                self._frame = f; self._n += 1; self._t_last = time.time(); self._cv.notify_all()

    def isOpened(self) -> bool:
        return self.size is not None and self.proc.poll() is None

    def read(self):
        with self._cv:
            if self._n == self._given:
                self._cv.wait(timeout=1.0)
            if self._n == self._given or self._frame is None:
                return False, None
            self._given = self._n
            return True, self._frame

    def get(self, prop, *a):
        if cv2 is not None and self.size is not None:
            if prop == cv2.CAP_PROP_FRAME_WIDTH: return float(self.size[0])
            if prop == cv2.CAP_PROP_FRAME_HEIGHT: return float(self.size[1])
        return 0.0

    def set(self, *a):
        return False

    def release(self) -> None:
        if self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=2)
            except Exception:
                self.proc.kill()


def open_udp(port: int, sniffer=sniff, popen=subprocess.Popen, opener=None):
    """Return a capture for the UDP stream on `port` or raise RuntimeError with a plain-words reason (injectable parts for tests)."""
    if cv2 is None and opener is None:
        raise RuntimeError("OpenCV (cv2) is not installed")
    kind, n = sniffer(port)
    if n == 0:
        ips = ", ".join(local_addresses()) or "this PC's address"
        raise RuntimeError(f"no data on UDP port {port}: set the streamer's destination to this PC ({ips}) port {port}, and join the streamer's Wi-Fi")
    if kind == "rtp":
        raise RuntimeError("the stream is RTP: choose raw H.264/H.265 or MPEG-TS in the streamer (RTP needs an SDP file)")
    if kind == "unknown":
        raise RuntimeError(f"UDP data on port {port} is not H.264, H.265 or MPEG-TS ({n} packets seen)")
    if DIRECT_DECODE and opener is None and popen is subprocess.Popen and kind in ("h264", "hevc", "h265"):
        try:
            return AvCapture(port, kind)
        except ImportError:
            pass                                             # no PyAV: the ffmpeg pipe decoder below
    if DIRECT_DECODE and opener is None and popen is subprocess.Popen and shutil.which("ffmpeg"):
        cap = PipeCapture("mpegts" if kind == "mpegts" else kind, port, shutil.which("ffmpeg"))
        if not cap.isOpened():
            cap.release()
            raise RuntimeError(f"{kind} stream seen ({n} packets) but no picture decoded: wrong codec setting in the streamer, or waiting for a key frame")
        return cap
    opener = opener or _open_cv
    if kind == "mpegts":
        return RelayedCapture(opener(f"udp://0.0.0.0:{port}?fifo_size=1000000&overrun_nonfatal=1"), None)
    ff = shutil.which("ffmpeg")
    if not ff:
        raise RuntimeError("ffmpeg is not installed (sudo apt install ffmpeg): needed to read raw H.264/H.265")
    local = _free_port()
    cmd = [ff, "-hide_banner", "-loglevel", "error", "-fflags", "nobuffer", "-flags", "low_delay", "-probesize", "32768", "-analyzeduration", "0",
           "-use_wallclock_as_timestamps", "1", "-f", kind, "-i", f"udp://0.0.0.0:{port}?fifo_size=1000000&overrun_nonfatal=1",
           "-c", "copy", "-f", "mpegts", f"udp://127.0.0.1:{local}?pkt_size=1316"]
    proc = popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(0.4)
    if proc.poll() is not None:
        raise RuntimeError("the ffmpeg helper stopped at once (is another program using UDP port %d?)" % port)
    cap = opener(f"udp://127.0.0.1:{local}")
    if cap is None or not cap.isOpened():
        RelayedCapture(cap, proc).release()
        raise RuntimeError(f"raw {kind} stream seen ({n} packets) but no picture decoded: wrong codec setting in the streamer, or waiting for a key frame")
    return RelayedCapture(cap, proc)
