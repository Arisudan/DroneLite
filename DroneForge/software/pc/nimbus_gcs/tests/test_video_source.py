import os
import tempfile
import time
import unittest

import numpy as np

from nimbus_gcs import video_source as VS

_SAFE_STATE = os.path.join(tempfile.mkdtemp(prefix="df_video_"), "video.json")


def wait(cond, t=3.0):
    end = time.monotonic() + t
    while time.monotonic() < end:
        if cond(): return True
        time.sleep(0.01)
    return False


FRAME = np.full((24, 32, 3), 7, np.uint8)


class TestSource(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(); os.environ["DRONEFORGE_VIDEO_STATE"] = os.path.join(self.tmp, "v.json")

    def tearDown(self):
        os.environ["DRONEFORGE_VIDEO_STATE"] = _SAFE_STATE          # never fall back to the owner's real file (a test that saves a source would overwrite it)

    def test_connect_live_stall_reconnect(self):
        mode = {"m": "ok", "opens": 0}
        def opener(k, t):
            mode["opens"] += 1
            return VS.FakeCapture(lambda: (FRAME if mode["m"] == "ok" else (time.sleep(0.05) or FRAME if mode["m"] == "slow" else None)))
        s = VS.VideoSource(opener=opener, stall_s=0.2, backoff=(0.05,), load=False)
        s.set_source("network", "rtsp://a"); s.start()
        self.assertTrue(wait(lambda: s.state == "live")); self.assertGreater(s.frame_count, 0)
        self.assertEqual(s.latest().shape, (24, 32, 3))
        s._last_t -= 1.0                                    # frames stop arriving
        self.assertEqual(s.state, "stalled")
        mode["m"] = "bad"                                   # read failures -> error -> reconnect
        self.assertTrue(wait(lambda: s.state.startswith("error") or mode["opens"] > 1))
        mode["m"] = "ok"
        self.assertTrue(wait(lambda: s.state == "live" and mode["opens"] > 1))
        s.stop(); self.assertEqual(s.state, "stopped"); self.assertIsNone(s.latest())

    def test_open_error_retries(self):
        n = {"c": 0}
        def opener(k, t):
            n["c"] += 1; return VS.FakeCapture(lambda: FRAME, opened=n["c"] > 2)
        s = VS.VideoSource(opener=opener, backoff=(0.02,), load=False)
        s.set_source("network", "x"); s.start()
        self.assertTrue(wait(lambda: s.state.startswith("error") or n["c"] >= 2))
        self.assertTrue(wait(lambda: s.state == "live")); s.stop()

    def test_no_url_is_error_not_thread(self):
        s = VS.VideoSource(opener=lambda k, t: None, load=False)
        s.set_source("network", ""); s.start()
        self.assertTrue(s.state.startswith("error")); self.assertFalse(s.running)

    def test_persistence_roundtrip(self):
        s = VS.VideoSource(opener=lambda k, t: None, load=False)
        s.set_source("network", "http://h/mjpeg", analog=False)
        s2 = VS.VideoSource(opener=lambda k, t: None)
        self.assertEqual((s2.kind, s2.target, s2.analog), ("network", "http://h/mjpeg", False))
        s.set_source("usb", "/dev/video2", analog=True)
        self.assertTrue(VS.load_choice()["analog"])

    def test_bad_state_file_is_safe(self):
        open(os.environ["DRONEFORGE_VIDEO_STATE"], "w").write("{nope")
        self.assertEqual(VS.load_choice()["kind"], "none")
        os.environ["DRONEFORGE_VIDEO_STATE"] = "/proc/nope/x.json"
        VS.save_choice("none", "")                          # must not raise

    def test_snapshot(self):
        s = VS.VideoSource(opener=lambda k, t: None, load=False)
        p = os.path.join(self.tmp, "a", "s.png")
        self.assertFalse(s.snapshot_png(p))
        s._frame = FRAME; self.assertTrue(s.snapshot_png(p)); self.assertGreater(os.path.getsize(p), 0)

    def test_cv2_missing(self):
        orig = VS.cv2; VS.cv2 = None
        try:
            s = VS.VideoSource(load=False); s.set_source("network", "rtsp://x"); s.start()
            self.assertIn("OpenCV not installed", s.state); self.assertFalse(s.running)
        finally:
            VS.cv2 = orig


if __name__ == "__main__":
    unittest.main()


class TestRtsp(unittest.TestCase):
    """RTSP (CosmoStreamer's RTSP server): low-latency options, RTP over UDP first and TCP as the fallback. No camera or network needed."""

    def test_tcp_is_tried_first_then_udp_and_the_environment_is_restored(self):
        import os
        from nimbus_gcs import video_source as V
        seen = []
        def opener(url):
            seen.append(os.environ.get("OPENCV_FFMPEG_CAPTURE_OPTIONS")); return V.FakeCapture(lambda: None, opened=len(seen) >= 2)
        old = os.environ.pop("OPENCV_FFMPEG_CAPTURE_OPTIONS", None); keep = V._OWNER_FFMPEG_OPTIONS; V._OWNER_FFMPEG_OPTIONS = None
        try:
            cap = V.open_rtsp("rtsp://192.168.50.1:554/video", opener=opener)
            self.assertTrue(cap.isOpened()); self.assertEqual(len(seen), 2)
            self.assertTrue(seen[0].startswith("rtsp_transport;tcp|")); self.assertTrue(seen[1].startswith("rtsp_transport;udp|"))
            for o in seen:
                for want in ("fflags;nobuffer", "flags;low_delay", "analyzeduration;0", "max_delay;0", "reorder_queue_size;0"):
                    self.assertIn(want, o)
            self.assertNotIn("OPENCV_FFMPEG_CAPTURE_OPTIONS", os.environ)                  # put back
        finally:
            V._OWNER_FFMPEG_OPTIONS = keep
            if old is not None: os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = old

    def test_the_first_transport_can_be_chosen_with_an_environment_variable(self):
        import os
        from nimbus_gcs import video_source as V
        old = os.environ.get("DRONEFORGE_RTSP_TRANSPORT")
        try:
            os.environ["DRONEFORGE_RTSP_TRANSPORT"] = "udp"; self.assertEqual(V.rtsp_transports(), ("udp", "tcp"))
            os.environ["DRONEFORGE_RTSP_TRANSPORT"] = "nonsense"; self.assertEqual(V.rtsp_transports(), ("tcp", "udp"))
            os.environ.pop("DRONEFORGE_RTSP_TRANSPORT"); self.assertEqual(V.rtsp_transports(), ("tcp", "udp"))
        finally:
            if old is not None: os.environ["DRONEFORGE_RTSP_TRANSPORT"] = old

    def test_an_owner_set_option_string_wins_and_no_second_try_is_made(self):
        import os
        from nimbus_gcs import video_source as V
        seen = []
        keep = V._OWNER_FFMPEG_OPTIONS; V._OWNER_FFMPEG_OPTIONS = "rtsp_transport;tcp"
        try:
            V.open_rtsp("rtsp://x/video", opener=lambda u: (seen.append(os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"]), V.FakeCapture(lambda: None, opened=False))[1])
            self.assertEqual(seen, ["rtsp_transport;tcp"])
        finally:
            V._OWNER_FFMPEG_OPTIONS = keep

    def test_rtsp_urls_go_to_open_rtsp(self):
        from nimbus_gcs import video_source as V
        called = []
        keep = V.open_rtsp; V.open_rtsp = lambda u, *a, **k: called.append(u) or "cap"; keep_d = V.RTSP_DIRECT; V.RTSP_DIRECT = False     # the OpenCV fallback path
        try:
            self.assertEqual(V.default_open("network", "rtsp://192.168.50.1:554/video"), "cap")
            self.assertEqual(called, ["rtsp://192.168.50.1:554/video"])
        finally:
            V.open_rtsp = keep; V.RTSP_DIRECT = keep_d

    def test_rtsp_urls_go_to_the_direct_client_first(self):
        from nimbus_gcs import video_source as V, rtsp_direct as R
        keep = R.RtspDirectCapture; R.RtspDirectCapture = lambda u: ("direct", u)
        try:
            self.assertEqual(V.default_open("network", "rtsp://192.168.50.1:554/video"), ("direct", "rtsp://192.168.50.1:554/video"))
        finally:
            R.RtspDirectCapture = keep


class TestDecoderThreads(unittest.TestCase):
    """OpenCV's default multi-threaded H.264 decoding holds back about one frame per thread (measured: 307 ms default vs 74 ms with one thread at 30 fps): the GCS opens network video with one."""

    def test_open_capture_passes_one_decoder_thread_by_default_and_the_variable_overrides(self):
        import os
        from nimbus_gcs import video_source as V
        seen = []
        keep = V.cv2.VideoCapture; V.cv2.VideoCapture = lambda *a: (seen.append(a), "cap")[1]
        old = os.environ.pop("DRONEFORGE_VIDEO_THREADS", None)
        try:
            self.assertEqual(V.open_capture("rtsp://x/video"), "cap")
            self.assertEqual(seen[-1], ("rtsp://x/video", V.cv2.CAP_FFMPEG, [V.cv2.CAP_PROP_N_THREADS, 1]))
            os.environ["DRONEFORGE_VIDEO_THREADS"] = "3"; V.open_capture("rtsp://x/video"); self.assertEqual(seen[-1][2], [V.cv2.CAP_PROP_N_THREADS, 3])
            os.environ["DRONEFORGE_VIDEO_THREADS"] = "0"; V.open_capture("rtsp://x/video"); self.assertEqual(seen[-1], ("rtsp://x/video", V.cv2.CAP_FFMPEG))     # 0 = OpenCV's default
            os.environ["DRONEFORGE_VIDEO_THREADS"] = "nonsense"; self.assertEqual(V.decoder_threads(), 1)
        finally:
            V.cv2.VideoCapture = keep
            os.environ.pop("DRONEFORGE_VIDEO_THREADS", None)
            if old is not None: os.environ["DRONEFORGE_VIDEO_THREADS"] = old

    def test_rtsp_and_udp_paths_both_use_it(self):
        import inspect
        from nimbus_gcs import video_source as V, video_udp as U
        self.assertIn("open_capture", inspect.getsource(V.open_rtsp)); self.assertIn("open_capture", inspect.getsource(V.default_open)); self.assertIn("open_capture", inspect.getsource(U._open_cv))

