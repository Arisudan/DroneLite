"""rtsp_direct: RTP unpacking (H.264 / H.265) and the whole RTSP session against a stand-in server that streams FFmpeg's own RTP packets (TCP interleaved)."""
import socket
import threading
import time
import unittest

import numpy as np

from nimbus_gcs.rtsp_direct import RtpDepack, RtspDirectCapture, parse_sdp


def rtp(seq, payload, marker=False, pt=96):
    return bytes([0x80, (0x80 if marker else 0) | pt, seq >> 8 & 0xFF, seq & 0xFF, 0, 0, 0, 0, 0, 0, 0, 1]) + payload


class TestDepack(unittest.TestCase):
    def test_single_stap_fu(self):
        d = RtpDepack("h264")
        self.assertIsNone(d.push(rtp(1, b"\x67SPS")))
        self.assertIsNone(d.push(rtp(2, b"\x18" + b"\x00\x03\x68PP" + b"\x00\x02\x06S")))            # STAP-A: PPS + SEI
        self.assertIsNone(d.push(rtp(3, b"\x7c\x85" + b"AAA")))                                      # FU-A start, IDR (type 5)
        au = d.push(rtp(4, b"\x7c\x45" + b"BBB", marker=True))                                       # FU-A end + marker: the picture is out NOW
        self.assertEqual(au, b"\x00\x00\x00\x01\x67SPS\x00\x00\x00\x01\x68PP\x00\x00\x00\x01\x06S\x00\x00\x00\x01\x65AAABBB")

    def test_lost_packet_drops_that_picture_only(self):
        d = RtpDepack("h264")
        d.push(rtp(1, b"\x7c\x85A")); self.assertIsNone(d.push(rtp(3, b"\x7c\x45B", marker=True)))   # seq 2 missing
        self.assertEqual(d.push(rtp(4, b"\x41X", marker=True)), b"\x00\x00\x00\x01\x41X")

    def test_h265_fu(self):
        d = RtpDepack("h265")
        d.push(rtp(1, bytes([49 << 1, 1, 0x80 | 19]) + b"AA"))
        self.assertEqual(d.push(rtp(2, bytes([49 << 1, 1, 0x40 | 19]) + b"BB", marker=True)), b"\x00\x00\x00\x01" + bytes([19 << 1, 1]) + b"AABB")

    def test_sdp(self):
        sdp = "v=0\r\nm=audio 0 RTP/AVP 97\r\na=control:trackID=1\r\nm=video 0 RTP/AVP 96\r\na=rtpmap:96 H264/90000\r\na=fmtp:96 packetization-mode=1;sprop-parameter-sets=Z0IAHg==,aM4G4g==\r\na=control:trackID=0\r\n"
        codec, pt, ctrl, ps = parse_sdp(sdp, "rtsp://192.168.50.1:554/video")
        self.assertEqual((codec, pt, ctrl), ("h264", 96, "rtsp://192.168.50.1:554/video/trackID=0"))
        self.assertTrue(ps.startswith(b"\x00\x00\x00\x01\x67"))


class FakeCosmo:
    """RTSP server stand-in: answers the session, then forwards the RTP packets FFmpeg's RTP muxer sends to a local UDP port, interleaved on channel 0."""

    SDP = "v=0\r\no=- 0 0 IN IP4 127.0.0.1\r\ns=t\r\nt=0 0\r\nm=video 0 RTP/AVP 96\r\na=rtpmap:96 H264/90000\r\na=fmtp:96 packetization-mode=1\r\na=control:trackID=0\r\n"

    def __init__(self):
        self.ls = socket.socket(); self.ls.bind(("127.0.0.1", 0)); self.ls.listen(1); self.port = self.ls.getsockname()[1]
        self.us = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); self.us.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20); self.us.bind(("127.0.0.1", 0)); self.uport = self.us.getsockname()[1]; self.us.settimeout(0.5)
        self.methods = []; self.stop = False
        threading.Thread(target=self._serve, daemon=True).start()

    def _serve(self):
        c, _ = self.ls.accept(); f = c.makefile("rb")
        while True:
            line = f.readline().decode()
            if not line: return
            method = line.split()[0]; hd = {}
            while True:
                ln = f.readline().decode().strip()
                if not ln: break
                k, v = ln.split(":", 1); hd[k.lower()] = v.strip()
            self.methods.append(method); body = self.SDP if method == "DESCRIBE" else ""
            extra = "Content-Type: application/sdp\r\n" if body else ""
            if method == "SETUP": extra += "Session: 12345;timeout=60\r\nTransport: RTP/AVP/TCP;unicast;interleaved=0-1\r\n"
            c.sendall(f"RTSP/1.0 200 OK\r\nCSeq: {hd['cseq']}\r\n{extra}Content-Length: {len(body)}\r\n\r\n{body}".encode())
            if method == "PLAY": break
        while not self.stop:
            try: p = self.us.recv(65536)
            except socket.timeout: continue
            try: c.sendall(b"$\x00" + len(p).to_bytes(2, "big") + p)
            except OSError: return


class TestSession(unittest.TestCase):
    def test_rtsp_session_and_latency(self):
        import av
        srv = FakeCosmo(); W, H = 640, 480; done = threading.Event()

        def publish():
            o = av.open(f"rtp://127.0.0.1:{srv.uport}?pkt_size=1200", "w", format="rtp")
            s = o.add_stream("libx264", rate=30); s.width, s.height, s.pix_fmt = W, H, "yuv420p"; s.options = {"preset": "ultrafast", "tune": "zerolatency", "g": "15"}
            rng = np.random.default_rng(0); bg = rng.integers(0, 255, (H * 2, W * 2, 3), np.uint8); t0 = time.time(); i = 0
            while not done.is_set() and i < 300:
                i += 1; img = bg[(i * 23) % H:(i * 23) % H + H, (i * 37) % W:(i * 37) % W + W].copy(); ms = int(time.time() * 1000) & 0xFFFFFFFF
                for b in range(32): img[0:40, b * 20:(b + 1) * 20] = 255 if (ms >> b) & 1 else 0
                for p in s.encode(av.VideoFrame.from_ndarray(img, format="bgr24")): o.mux(p)
                time.sleep(max(0, t0 + i / 30 - time.time()))
            o.close()

        threading.Thread(target=publish, daemon=True).start()
        cap = RtspDirectCapture(f"rtsp://127.0.0.1:{srv.port}/video")
        lat = []; t0 = time.time(); n = 0; f = None
        while time.time() - t0 < 4:
            ok, fr = cap.read()
            if not ok: continue
            f = fr; n += 1; now = int(time.time() * 1000) & 0xFFFFFFFF; ms = sum(1 << b for b in range(32) if f[20, b * 20 + 10].mean() > 128)
            if time.time() - t0 > 1: lat.append((now - ms) & 0xFFFFFFFF)
        done.set(); srv.stop = True; cap.release()
        self.assertEqual(srv.methods[:4], ["OPTIONS", "DESCRIBE", "SETUP", "PLAY"])
        self.assertIsNotNone(f, f"no picture ({cap.err})"); self.assertEqual(f.shape, (H, W, 3)); self.assertGreater(n, 90)                              # ~30 fps
        print(f"\nRTSP direct: {n / 4:.0f} fps, latency median {np.median(lat):.0f} ms, max {max(lat)} ms"); self.assertLess(float(np.median(lat)), 25.0)             # FFmpeg's RTSP reader: 40 ms on this stream; this stand-in server keeps Nagle ON (worst case)


if __name__ == "__main__":
    unittest.main()
