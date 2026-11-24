"""Video health line, display delay and the QR latency test (camera to GCS in ms). No camera, network or waiting: the clock and the decoder are injected."""
import os
import tempfile
import time
import unittest
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ["DRONEFORGE_VIDEO_STATE"] = os.path.join(tempfile.mkdtemp(), "video.json")

import numpy as np                                          # noqa: E402

from nimbus_gcs import video_health as H                    # noqa: E402
from nimbus_gcs import video_latency as L                   # noqa: E402
from nimbus_gcs.video_source import FakeCapture, VideoSource   # noqa: E402


class Clock:
    def __init__(self): self.t = 100.0
    def __call__(self): return self.t


def steady(h, c, fps=30.0, seconds=3.0):
    for _ in range(int(fps * seconds)):
        h.on_frame(c.t); c.t += 1.0 / fps


class TestHealthStates(unittest.TestCase):
    def test_steady_30_fps_is_live_with_no_jitter(self):
        c = Clock(); h = H.VideoHealth(c); steady(h, c); d = h.snapshot()
        self.assertEqual(d["state"], "LIVE"); self.assertAlmostEqual(d["fps"], 30.0, delta=0.6)
        self.assertAlmostEqual(d["interval_ms"], 33.3, delta=0.6); self.assertLess(d["jitter_ms"], 0.5)

    def test_the_first_frames_are_starting_not_degraded(self):
        c = Clock(); h = H.VideoHealth(c)
        for _ in range(3): h.on_frame(c.t); c.t += 0.033
        self.assertEqual(h.snapshot()["state"], "STARTING"); self.assertEqual(H.format_line(h.snapshot()), "STARTING")
        for _ in range(3): h.on_frame(c.t); c.t += 0.033
        self.assertEqual(h.snapshot()["state"], "LIVE")

    def test_a_long_gap_or_low_fps_is_degraded(self):
        c = Clock(); h = H.VideoHealth(c); steady(h, c, seconds=1.0); c.t += 0.4; steady(h, c, seconds=1.0)      # one 400 ms hole in the window
        d = h.snapshot(); self.assertEqual(d["state"], "DEGRADED"); self.assertGreater(d["interval_max_ms"], 350)
        c2 = Clock(); h2 = H.VideoHealth(c2); steady(h2, c2, fps=6.0, seconds=3.0); self.assertEqual(h2.snapshot()["state"], "DEGRADED")

    def test_frozen_then_no_signal_then_stopped_and_connecting(self):
        c = Clock(); h = H.VideoHealth(c); steady(h, c, seconds=1.0)
        c.t += 1.5; self.assertEqual(h.snapshot()["state"], "FROZEN")
        c.t += 2.0; self.assertEqual(h.snapshot()["state"], "NO SIGNAL")
        self.assertEqual(h.snapshot(running=False)["state"], "STOPPED")
        self.assertEqual(H.VideoHealth(c).snapshot(connecting=True)["state"], "CONNECTING")
        self.assertEqual(H.VideoHealth(c).snapshot()["state"], "NO SIGNAL")

    def test_the_line_starts_with_the_state_word_and_says_when_the_camera_delay_is_unmeasured(self):
        c = Clock(); h = H.VideoHealth(c); steady(h, c); h.on_shown(c.t - 0.021, c.t)
        line = H.format_line(h.snapshot())
        self.assertTrue(line.startswith("LIVE · 30 fps")); self.assertIn("drawn 21 ms after arrival", line); self.assertIn("not measured", line)
        h.on_e2e(182.0, c.t); h.on_e2e(170.0, c.t); line = H.format_line(h.snapshot())
        self.assertIn("camera to GCS 170 ms", line); self.assertIn("min 170", line); self.assertIn("max 182", line); self.assertIn("2 readings", line); self.assertIn("camera to screen 191 ms", line)
        self.assertNotIn("not measured", line)
        for st, word in (("STOPPED", "STOPPED"),):
            self.assertEqual(H.format_line({"state": st}), word)
        self.assertTrue(H.format_line({"state": "FROZEN", "age_s": 1.7}).startswith("FROZEN"))


class TestQrLatency(unittest.TestCase):
    def test_the_code_round_trips_inside_a_larger_noisy_frame(self):
        code = L.render_code(1234567, scale=8)
        frame = np.full((720, 1280, 3), 90, np.uint8); frame += np.random.RandomState(1).randint(0, 25, frame.shape, dtype=np.uint8)
        y, x = 200, 400; frame[y:y + code.shape[0], x:x + code.shape[1]] = code[:code.shape[0], :code.shape[1]] if y + code.shape[0] <= 720 else code[:520]
        self.assertEqual(L.decode_code(frame), 1234567)

    def test_a_frame_without_a_code_gives_none(self):
        self.assertIsNone(L.decode_code(np.full((360, 640, 3), 128, np.uint8)))
        self.assertIsNone(L.decode_code(None))

    def test_delay_is_the_clock_difference_and_survives_the_wrap(self):
        t = 5000.000; sent = L.clock_ms(t - 0.180); self.assertAlmostEqual(L.delay_ms(sent, t), 180.0, delta=1.0)
        t = L.MOD_MS / 1000.0 + 0.050; sent = L.clock_ms(t - 0.120); self.assertAlmostEqual(L.delay_ms(sent, t), 120.0, delta=1.0)       # across the wrap of the modulo
        self.assertIsNone(L.delay_ms(L.clock_ms(10.0), 20.0))                                                                          # 10 s old: a stale or wrong code, dropped

    def test_the_probe_reports_delays_and_keeps_only_the_newest_frame(self):
        got = []; t_read = 1000.0
        probe = L.LatencyProbe(got.append, decoder=lambda f: L.clock_ms(t_read - 0.250))
        probe.start()
        for _ in range(3): probe.feed(np.zeros((8, 8, 3), np.uint8), t_read)
        t0 = time.monotonic()
        while not got and time.monotonic() - t0 < 2: time.sleep(0.01)
        probe.stop(); self.assertTrue(got); self.assertAlmostEqual(got[0], 250.0, delta=1.0); self.assertLessEqual(probe.attempts, 3); self.assertFalse(probe.running)

    def test_the_probe_ignores_frames_without_a_readable_code(self):
        got = []; probe = L.LatencyProbe(got.append, decoder=lambda f: None); probe.start(); probe.feed(np.zeros((8, 8, 3), np.uint8), 1.0); time.sleep(0.3); probe.stop()
        self.assertEqual(got, []); self.assertEqual(probe.found, 0)


class TestSourceHealth(unittest.TestCase):
    def test_the_source_feeds_the_health_tracker_and_records_the_display_delay(self):
        frame = np.zeros((48, 64, 3), np.uint8)
        src = VideoSource(opener=lambda k, t: FakeCapture(lambda: (time.sleep(0.02), frame.copy())[1]), load=False)
        src.set_source("network", "rtsp://x/video"); src.start()
        t0 = time.monotonic()
        while src.frame_count < 8 and time.monotonic() - t0 < 5: time.sleep(0.02)
        f, t = src.latest_with_time(); self.assertIsNotNone(f); self.assertLess(time.monotonic() - t, 1.0)
        time.sleep(0.01); src.note_shown(t); d = src.health_snapshot()
        src.stop()
        self.assertEqual(d["state"], "LIVE"); self.assertGreater(d["fps"], 20); self.assertGreaterEqual(d["display_ms"], 10.0)
        self.assertEqual(src.health_snapshot()["state"], "STOPPED")                                # stop() resets and says so


if __name__ == "__main__":
    unittest.main()
