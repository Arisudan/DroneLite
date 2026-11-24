"""The CosmoStreamer RTSP stream, read DIRECTLY: our own small RTSP client (OPTIONS, DESCRIBE, SETUP over TCP, PLAY) and RTP unpacking, decoded in this process (PyAV).

Why not FFmpeg's RTSP reader (OpenCV or PyAV): its stream parser releases a picture only when the NEXT picture starts arriving, so every frame waits one frame
period (33 ms at 30 fps) before it is even decoded; on a busy picture (fast motion) its one-by-one reading also fell behind. Here a picture is decoded the moment
its LAST RTP packet arrives (the RTP marker bit), and read() always hands out the NEWEST picture (2026-11-23, 1280x720 30 fps loopback: FFmpeg's reader 40 ms,
this one: see tests/test_rtsp_direct.py and HANDOFF 5e).

RTP over the RTSP TCP connection (interleaved): no packet loss on Wi-Fi, one socket, no firewall ports. H.264 (RFC 6184) and H.265 (RFC 7798)."""
from __future__ import annotations

import base64
import re
import socket
import threading
import time
from typing import List, Optional, Tuple
from urllib.parse import urlparse

try:
    import cv2  # type: ignore
except Exception:                                                     # noqa: BLE001
    cv2 = None

SC = b"\x00\x00\x00\x01"
QUICKACK = getattr(socket, "TCP_QUICKACK", 0)


class RtpDepack:
    """RTP payloads -> whole access units (Annex B). push() returns the finished picture's bytes on the marker bit, else None. A lost packet drops that picture."""

    def __init__(self, codec: str = "h264") -> None:
        self.h265 = codec.lower() in ("h265", "hevc"); self.buf: List[bytes] = []; self.seq: Optional[int] = None; self.bad = False; self.fu = False; self.lost = 0

    def push(self, pkt: bytes) -> Optional[bytes]:
        if len(pkt) < 12 or (pkt[0] >> 6) != 2: return None
        cc = pkt[0] & 0x0F; marker = bool(pkt[1] & 0x80); seq = (pkt[2] << 8) | pkt[3]; off = 12 + 4 * cc
        if pkt[0] & 0x10:                                              # header extension
            if len(pkt) < off + 4: return None
            off += 4 + 4 * ((pkt[off + 2] << 8) | pkt[off + 3])
        end = len(pkt) - (pkt[-1] if pkt[0] & 0x20 else 0)             # padding
        if self.seq is not None and seq != (self.seq + 1) & 0xFFFF: self.bad = True; self.lost += 1
        self.seq = seq; p = pkt[off:end]
        if p: (self._h265 if self.h265 else self._h264)(p)
        if not marker: return None
        au = b"".join(self.buf) if (self.buf and not self.bad) else None
        self.buf = []; self.bad = False; self.fu = False
        return au

    def _h264(self, p: bytes) -> None:
        t = p[0] & 0x1F
        if 1 <= t <= 23: self.buf += [SC, p]
        elif t == 24:                                                  # STAP-A
            i = 1
            while i + 2 <= len(p):
                n = (p[i] << 8) | p[i + 1]; self.buf += [SC, p[i + 2:i + 2 + n]]; i += 2 + n
        elif t == 28 and len(p) > 2:                                   # FU-A
            if p[1] & 0x80: self.buf += [SC, bytes([(p[0] & 0xE0) | (p[1] & 0x1F)])]; self.fu = True
            elif not self.fu: self.bad = True; return
            self.buf.append(p[2:])
            if p[1] & 0x40: self.fu = False

    def _h265(self, p: bytes) -> None:
        if len(p) < 3: return
        t = (p[0] >> 1) & 0x3F
        if t < 48: self.buf += [SC, p]
        elif t == 48:                                                  # aggregation packet
            i = 2
            while i + 2 <= len(p):
                n = (p[i] << 8) | p[i + 1]; self.buf += [SC, p[i + 2:i + 2 + n]]; i += 2 + n
        elif t == 49 and len(p) > 3:                                   # fragmentation unit
            if p[2] & 0x80: self.buf += [SC, bytes([(p[0] & 0x81) | ((p[2] & 0x3F) << 1), p[1]])]; self.fu = True
            elif not self.fu: self.bad = True; return
            self.buf.append(p[3:])
            if p[2] & 0x40: self.fu = False


def parse_sdp(sdp: str, base: str) -> Tuple[str, int, str, bytes]:
    """(codec, payload type, control URL, parameter sets in Annex B) of the first video stream."""
    codec, pt, ctrl, ps, in_video = "h264", 96, base, b"", False
    for ln in sdp.splitlines():
        ln = ln.strip()
        if ln.startswith("m="):
            in_video = ln.startswith("m=video")
            if in_video: pt = int(ln.split()[3])
        elif in_video and ln.startswith("a=rtpmap:"):
            m = re.match(r"a=rtpmap:(\d+)\s+([A-Za-z0-9]+)/", ln)
            if m and int(m.group(1)) == pt: codec = m.group(2).lower()
        elif in_video and ln.startswith("a=control:"):
            c = ln[len("a=control:"):]
            ctrl = c if c.startswith("rtsp://") else (base if c == "*" else base.rstrip("/") + "/" + c)
        elif in_video and ln.startswith("a=fmtp:"):
            for k in ("sprop-parameter-sets", "sprop-vps", "sprop-sps", "sprop-pps"):
                for v in re.findall(k + r"=([^;\s]+)", ln):
                    for b in v.split(","):
                        try: ps += SC + base64.b64decode(b + "=" * (-len(b) % 4))
                        except Exception: pass                          # noqa: BLE001
    return ("h265" if codec in ("h265", "hevc") else "h264"), pt, ctrl, ps


class RtspDirectCapture:
    """cv2-like capture (isOpened / read / get / set / release) for VideoSource."""

    def __init__(self, url: str, timeout: float = 5.0) -> None:
        import av
        self._av = av; self.url = url; self.size = None; self.proc = None; self.cseq = 0; self.session = ""
        u = urlparse(url); self.base = url.split("?")[0] if not u.username else url.replace(f"{u.username}:{u.password}@", "")
        self.s = socket.create_connection((u.hostname, u.port or 554), timeout=timeout); self.s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20); self.rx = b""
        self._req("OPTIONS", self.base)
        hd, sdp = self._req("DESCRIBE", self.base, {"Accept": "application/sdp"})
        base = hd.get("content-base", self.base)
        self.codec, self.pt, ctrl, ps = parse_sdp(sdp, base)
        hd, _ = self._req("SETUP", ctrl, {"Transport": "RTP/AVP/TCP;unicast;interleaved=0-1"})
        self.session = hd.get("session", "").split(";")[0]; m = re.search(r"interleaved=(\d+)", hd.get("transport", "")); self.ch = int(m.group(1)) if m else 0
        self._req("PLAY", self.base if ctrl == self.base else base, {"Range": "npt=0.000-"})
        self.cc = av.CodecContext.create("hevc" if self.codec == "h265" else "h264", "r"); self.cc.thread_type = "SLICE"
        self.cc.flags |= av.codec.context.Flags.low_delay
        self.dep = RtpDepack(self.codec); self.ps = ps
        self._cv = threading.Condition(); self._fr = None; self._n = 0; self._given = 0; self._open = True; self.err = None; self.decode_ms = 0.0
        self.s.settimeout(1.5)
        threading.Thread(target=self._run, name="rtsp-direct", daemon=True).start()

    # ---- RTSP requests (before PLAY: plain request / response)
    def _req(self, method: str, url: str, extra: dict = None, keepalive: bool = False) -> Tuple[dict, str]:
        self.cseq += 1; h = {"CSeq": str(self.cseq), "User-Agent": "DroneForge-GCS"}
        if self.session: h["Session"] = self.session
        h.update(extra or {})
        self.s.sendall((f"{method} {url} RTSP/1.0\r\n" + "".join(f"{k}: {v}\r\n" for k, v in h.items()) + "\r\n").encode())
        if keepalive: return {}, ""                                    # after PLAY: the reply arrives in the RTP stream and is skipped there
        while b"\r\n\r\n" not in self.rx:
            d = self.s.recv(65536)
            if not d: raise ConnectionError(f"RTSP {method}: connection closed")
            self.rx += d
        head, self.rx = self.rx.split(b"\r\n\r\n", 1); lines = head.decode("latin-1").split("\r\n")
        st = lines[0].split(" ", 2); hd = {}
        for ln in lines[1:]:
            if ":" in ln: k, v = ln.split(":", 1); hd[k.strip().lower()] = v.strip()
        n = int(hd.get("content-length", "0"))
        while len(self.rx) < n:
            d = self.s.recv(65536)
            if not d: raise ConnectionError(f"RTSP {method}: connection closed")
            self.rx += d
        body, self.rx = self.rx[:n].decode("utf-8", "ignore"), self.rx[n:]
        if len(st) < 2 or st[1] != "200": raise ConnectionError(f"RTSP {method}: {' '.join(st[1:])}")
        return hd, body

    # ---- stream
    def _run(self) -> None:
        buf = self.rx; last_ka = time.monotonic(); first = True
        try:
            while self._open:
                if time.monotonic() - last_ka > 20.0:                      # keep the session alive (servers drop it after ~60 s of silence)
                    try: self._req("OPTIONS", self.base, keepalive=True)
                    except OSError: pass
                    last_ka = time.monotonic()
                while len(buf) >= 4:
                    if buf[0] == 0x24:                                     # '$' channel length(2) RTP
                        n = (buf[2] << 8) | buf[3]
                        if len(buf) < 4 + n: break
                        ch, pkt, buf = buf[1], buf[4:4 + n], buf[4 + n:]
                        if ch == self.ch:
                            au = self.dep.push(pkt)
                            if au: self._decode((self.ps + au) if first else au); first = False
                    elif buf.startswith(b"RTSP/"):                         # a reply (keep-alive): skip its header and body
                        i = buf.find(b"\r\n\r\n")
                        if i < 0: break
                        m = re.search(rb"content-length:\s*(\d+)", buf[:i], re.I); n = int(m.group(1)) if m else 0
                        if len(buf) < i + 4 + n: break
                        buf = buf[i + 4 + n:]
                    else:                                                  # lost sync: find the next '$'
                        j = buf.find(b"$", 1); buf = buf[j:] if j > 0 else b""
                try:
                    d = self.s.recv(262144)
                    if QUICKACK: self.s.setsockopt(socket.IPPROTO_TCP, QUICKACK, 1)    # ACK at once, every time (Linux clears it): a server with Nagle on otherwise holds the
                except socket.timeout:                                                 # rest of each picture until our delayed ACK (~40 ms; measured 37 ms -> 6 ms)
                    raise ConnectionError("no video for 1.5 s")
                if not d: raise ConnectionError("CosmoStreamer closed the connection")
                buf += d
        except Exception as e:                                             # noqa: BLE001
            self.err = e
        with self._cv: self._open = False; self._cv.notify_all()

    def _decode(self, au: bytes) -> None:
        t = time.perf_counter()
        try:
            for fr in self.cc.decode(self._av.Packet(au)):
                with self._cv: self._fr = fr; self._n += 1; self._cv.notify_all()
        except self._av.error.FFmpegError:                                 # damaged picture: the next key frame recovers
            pass
        self.decode_ms = 1000 * (time.perf_counter() - t)

    # ---- cv2-like
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
        was = self._open; self._open = False
        try:
            if was: self.s.sendall(f"TEARDOWN {self.base} RTSP/1.0\r\nCSeq: {self.cseq + 1}\r\nSession: {self.session}\r\n\r\n".encode())
        except OSError:
            pass
        try: self.s.close()
        except OSError: pass
