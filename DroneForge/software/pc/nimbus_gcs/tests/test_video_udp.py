"""UDP video input: address parsing, stream classification, and (when ffmpeg exists) a real raw-H.264 stream through the GCS path."""
import os, shutil, socket, subprocess, sys, time, unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
from nimbus_gcs import video_udp as V


def sc(nal: bytes) -> bytes:
    return b"\x00\x00\x00\x01" + nal


class TestParse(unittest.TestCase):
    def test_addresses(self):
        for t, p in (("udp://0.0.0.0:3000", 3000), ("3000", 3000), ("192.168.50.65:3000", 3000), ("udp://192.168.50.65:5600?x=1", 5600)):
            self.assertEqual(V.parse_udp_target(t), p, t)
        for t in ("rtsp://1.2.3.4:554/x", "http://host:8080/v", "", "/dev/video0", "abc"):
            self.assertIsNone(V.parse_udp_target(t), t)


class TestClassify(unittest.TestCase):
    def test_kinds(self):
        ts = [b"\x47" + bytes(187)] * 7 + [(b"\x47" + bytes(187)) * 7]
        self.assertEqual(V.classify(ts), "mpegts")
        h264 = [sc(b"\x67\x42\x00\x1f") + bytes(40), sc(b"\x68\xce\x3c\x80") + bytes(8), sc(b"\x65\x88\x84") + bytes(1300)]
        self.assertEqual(V.classify(h264), "h264")
        hevc = [sc(b"\x40\x01\x0c\x01") + bytes(20), sc(b"\x42\x01\x01\x01") + bytes(30), sc(b"\x44\x01\xc1\x72") + bytes(8), sc(b"\x26\x01\xaf") + bytes(1300)]
        self.assertEqual(V.classify(hevc), "hevc")
        rtp = [bytes([0x80, 96, 0, i, 0, 0, 0, i, 0, 0, 0, 1]) + bytes(100) for i in range(10)]
        self.assertEqual(V.classify(rtp), "rtp")
        self.assertEqual(V.classify([]), "unknown")
        self.assertEqual(V.classify([bytes(100)] * 5), "unknown")


class TestOpen(unittest.TestCase):
    def test_no_data_message_names_the_pc(self):
        with self.assertRaises(RuntimeError) as c:
            V.open_udp(3000, sniffer=lambda port: ("unknown", 0), opener=lambda u: None)
        self.assertIn("no data on UDP port 3000", str(c.exception))

    def test_rtp_and_unknown_refused_in_words(self):
        for k, word in (("rtp", "RTP"), ("unknown", "not H.264")):
            with self.assertRaises(RuntimeError) as c:
                V.open_udp(3000, sniffer=lambda port, k=k: (k, 20), opener=lambda u: None)
            self.assertIn(word, str(c.exception))

    def test_mpegts_opens_directly_without_helper(self):
        seen = []
        class Cap:
            def isOpened(self): return True
            def release(self): pass
        cap = V.open_udp(3000, sniffer=lambda port: ("mpegts", 30), opener=lambda u: seen.append(u) or Cap())
        self.assertIsNone(cap.proc); self.assertTrue(seen[0].startswith("udp://0.0.0.0:3000"))

    @unittest.skipUnless(shutil.which("ffmpeg") and V.cv2 is not None, "needs ffmpeg and OpenCV")
    def test_real_raw_h264_stream_end_to_end(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.bind(("127.0.0.1", 0)); port = s.getsockname()[1]; s.close()
        gen = subprocess.Popen(["ffmpeg", "-hide_banner", "-loglevel", "error", "-re", "-f", "lavfi", "-i", "testsrc=size=640x360:rate=30", "-c:v", "libx264",
                                "-preset", "ultrafast", "-tune", "zerolatency", "-bf", "0", "-g", "15", "-f", "h264", f"udp://127.0.0.1:{port}?pkt_size=1400"],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            time.sleep(1.0)
            cap = V.open_udp(port)
            n = 0; t0 = time.time()
            while time.time() - t0 < 4 and n < 20:
                ok, f = cap.read()
                n += int(ok)
            self.assertGreaterEqual(n, 20); self.assertEqual(f.shape, (360, 640, 3))
            proc = cap.proc; cap.release()
            if proc is not None: self.assertIsNotNone(proc.poll())      # a helper process (relay / pipe decoder) is stopped with the capture
            else: self.assertFalse(cap.isOpened())                     # the in-process decoder (AvCapture, 2026-11-21) has none: closed
        finally:
            gen.terminate()


if __name__ == "__main__":
    unittest.main()
