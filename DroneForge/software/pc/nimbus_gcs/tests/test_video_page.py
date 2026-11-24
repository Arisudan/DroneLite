import os
import tempfile
import time
import unittest
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ["DRONEFORGE_VIDEO_STATE"] = os.path.join(tempfile.mkdtemp(), "video.json")

import numpy as np
from PyQt6.QtWidgets import QApplication, QCheckBox, QMessageBox, QPushButton  # noqa: E402

from nimbus_gcs.main_window import apply_dark               # noqa: E402
from nimbus_gcs import video_page as VP                     # noqa: E402
VP.PRESET_FILE = os.path.join(tempfile.mkdtemp(), "video_preset.json")      # never the owner's saved SOURCE choice
from nimbus_gcs.video_source import VideoSource             # noqa: E402
from nimbus_gcs.video_pip import VideoPip                    # noqa: E402
from nimbus_gcs.video_source import FakeCapture              # noqa: E402
from nimbus_pc import vtx                                   # noqa: E402
from nimbus_pc.emulator import VTX_CONFIG                   # noqa: E402

_app = QApplication.instance() or QApplication([])
apply_dark(_app)


class FakeSess:
    def __init__(self, reply=True):
        self.sent = []; self.reply = reply

    def send_frame(self, f):
        self.sent.append(f)

    def request_msp(self, cmd, timeout=2.0):
        return {"cmd": cmd, "payload": VTX_CONFIG} if self.reply else None


def _win(connected=True, reply=True, decoded=None):
    sess = FakeSess(reply)
    model = SimpleNamespace(connected=connected, core=SimpleNamespace(sess=sess if connected else None),
                            state=SimpleNamespace(decoded=decoded or {}))
    return SimpleNamespace(model=model, video_source=VideoSource(opener=lambda k, t: None, load=False)), sess


class TestVideoPage(unittest.TestCase):
    def setUp(self):
        for n in ("question", "information", "warning", "critical"):
            setattr(QMessageBox, n, staticmethod(lambda *a, **k: (_ for _ in ()).throw(AssertionError("dialog"))))

    def test_builds_without_checkbox(self):
        win, _ = _win(); p = VP.VideoPage(win)
        self.assertEqual(p.findChildren(QCheckBox), [])

    def test_snapshot_writes_png(self):
        win, _ = _win(); p = VP.VideoPage(win)
        d = tempfile.mkdtemp(); VP.SNAP_DIR = d
        p.src._frame = np.full((40, 60, 3), 90, np.uint8)
        p.snapshot()
        files = os.listdir(d)
        self.assertEqual(len(files), 1); self.assertTrue(files[0].endswith("_snapshot.png"))
        self.assertIn("Saved", p.video_status.text())

    def test_snapshot_without_frame_inline(self):
        win, _ = _win(); p = VP.VideoPage(win); p.snapshot()
        self.assertIn("No frame", p.video_status.text())

    def test_live_frames_render(self):
        win, _ = _win(); p = VP.VideoPage(win)
        win.video_source._opener = lambda k, t: FakeCapture(lambda: np.zeros((48, 64, 3), np.uint8))
        p.url.setText("rtsp://x/y"); p.start_video()
        for _ in range(100):
            if win.video_source.state == "live" and win.video_source.frame_count >= 8: break
            time.sleep(0.02)
        p._tick(); p.view.resize(300, 200); p.view.grab()
        self.assertTrue(p.video_status.text().startswith("LIVE"), p.video_status.text())      # the health line: the state word first
        p.stop_video()


    def test_the_picture_poll_is_5_ms_and_a_new_frame_is_drawn_and_timed_without_waiting_for_the_slow_tick(self):
        win, _ = _win(); p = VP.VideoPage(win)
        self.assertEqual(p._vtimer.interval(), 5); self.assertEqual(p._timer.interval(), 200)
        win.video_source._opener = lambda k, t: FakeCapture(lambda: (time.sleep(0.02), np.full((48, 64, 3), 7, np.uint8))[1])
        p.url.setText("rtsp://x/y"); p.start_video()
        for _ in range(100):
            if win.video_source.frame_count >= 5: break
            time.sleep(0.02)
        p._view_tick()                                           # only the fast timer's work, not the 200 ms tick
        self.assertIsNotNone(p.view.frame)
        d = win.video_source.health_snapshot(); self.assertIsNotNone(d["display_ms"]); self.assertLess(d["display_ms"], 1000.0)
        p.stop_video()

    def test_the_latency_test_button_opens_the_code_window_and_runs_the_probe(self):
        win, _ = _win(); p = VP.VideoPage(win)
        self.assertEqual(p.btn_lat.text(), "Latency test"); self.assertFalse(p.probe.running)
        p.toggle_latency_test()
        self.assertTrue(p.lat_window.isVisible()); self.assertTrue(p.probe.running); self.assertEqual(p.btn_lat.text(), "Stop latency test")
        p.lat_window.resize(400, 480); self.assertFalse(p.lat_window.grab().isNull())          # the code paints
        p.toggle_latency_test()
        self.assertFalse(p.lat_window.isVisible()); self.assertFalse(p.probe.running); self.assertEqual(p.btn_lat.text(), "Latency test")
        self.assertEqual(p.findChildren(QCheckBox), []); self.assertEqual(p.lat_window.findChildren(QCheckBox), [])

    def test_calibrate_lens_flow_on_the_video_page(self):
        from nimbus_gcs.tests.test_lens_calib import _render
        cfg = open(VP.os.path.join(VP.os.path.dirname(VP.__file__), "..", "..", "..", "gazebo", "config", "o4_camera.yaml")).read()
        win, _ = _win(); p = VP.VideoPage(win)
        p.preset.setCurrentIndex(0); p._on_preset(0)                                         # Simulator: refused, said inline
        p.toggle_calibration(); self.assertIsNone(p.cal); self.assertIn("for the real O4", p.cal_status.text())
        p.preset.setCurrentIndex(1); p._on_preset(1)                                         # DJI O4
        frame = _render((640, 360), 0, 0.6, 0)
        win.video_source._opener = lambda k, t: FakeCapture(lambda: (time.sleep(0.03), frame)[1])
        p.start_video()
        for _ in range(100):
            if win.video_source.state == "live": break
            time.sleep(0.02)
        p.toggle_calibration()
        self.assertIsNotNone(p.cal); self.assertTrue(p.cal_window.isVisible()); self.assertEqual(p.btn_cal.text(), "Finish calibration")
        self.assertFalse(p.cal_window.grab().isNull())
        for _ in range(60):
            if p.cal.views: break
            time.sleep(0.05)
        self.assertEqual(len(p.cal.views), 1); p._tick(); self.assertIn("views 1/25", p.cal_status.text())
        p._view_tick(force=True); self.assertIsNotNone(p.view.frame)                        # the picture with the corners drawn
        p.toggle_calibration()                                                               # Finish with 1 view: refused, nothing saved
        for _ in range(40):
            if p.cal.state != "collecting": break
            time.sleep(0.05)
        p._tick(); self.assertEqual(p.cal.state, "failed"); self.assertIn("Nothing saved", p.cal_status.text()); self.assertFalse(p.cal_window.isVisible())
        self.assertEqual(open(VP.os.path.join(VP.os.path.dirname(VP.__file__), "..", "..", "..", "gazebo", "config", "o4_camera.yaml")).read(), cfg)
        p.stop_video()

    def test_a_degraded_or_frozen_video_is_not_shown_as_live(self):
        win, _ = _win(); p = VP.VideoPage(win); src = win.video_source
        src._opener = lambda k, t: FakeCapture(lambda: (time.sleep(0.3), np.zeros((48, 64, 3), np.uint8))[1])        # ~3 fps: far too slow
        p.url.setText("rtsp://x/y"); p.start_video()
        for _ in range(150):
            if src.frame_count >= 5: break
            time.sleep(0.02)
        p._tick(); txt = p.video_status.text(); src.stop()
        self.assertTrue(txt.startswith("DEGRADED"), txt)


class TestPip(unittest.TestCase):
    def test_hidden_without_source_and_shows_live(self):
        win, _ = _win(); pip = VideoPip(win); pip.refresh()
        self.assertFalse(pip.isVisibleTo(None) and pip.isVisible())
        self.assertTrue(pip.isHidden())
        src = win.video_source
        src._opener = lambda k, t: FakeCapture(lambda: np.zeros((48, 64, 3), np.uint8))
        src.set_source("network", "rtsp://x"); src.start()
        for _ in range(100):
            if src.state == "live": break
            time.sleep(0.02)
        pip.refresh(); self.assertFalse(pip.isHidden()); pip.grab()
        src.stop(); pip.refresh(); self.assertTrue(pip.isHidden())
        self.assertEqual(pip.findChildren(QCheckBox), []); self.assertEqual(pip.findChildren(QPushButton), [])


if __name__ == "__main__":
    unittest.main()
