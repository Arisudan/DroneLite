"""2026-11-24 GCS changes: map RST (restart the map pipeline, off the GUI thread), 3D mode tiles, the battery chip, and a video-source stop that never blocks the GUI."""
import os
import stat
import tempfile
import threading
import time
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
from PyQt6.QtWidgets import QApplication

_app = QApplication.instance() or QApplication([])


def spin(s):
    t0 = time.monotonic()
    while time.monotonic() - t0 < s:
        _app.processEvents(); time.sleep(0.01)


class TestMapRestart(unittest.TestCase):
    def test_rst_runs_stop_then_start_in_the_background_and_reenables(self):
        from nimbus_gcs.main_window import MainWindow
        from nimbus_gcs import rviz_map_view as R
        d = tempfile.mkdtemp(); log = os.path.join(d, "calls"); fake = os.path.join(d, "map.sh")
        open(fake, "w").write(f"#!/bin/sh\necho \"$@\" >> {log}\nsleep 0.3\necho ok\n"); os.chmod(fake, os.stat(fake).st_mode | stat.S_IEXEC)
        w = MainWindow(); w.show(); w.goto("FLY"); v = w.fly.flymap; keep = R.RvizMapView.MAP_SH
        try:
            R.RvizMapView.MAP_SH = fake
            t0 = time.monotonic(); v.restart_map(); self.assertLess(time.monotonic() - t0, 0.1)          # the GUI is not blocked
            self.assertEqual(v._rst_state, "running"); self.assertFalse(v.btn_rst.isEnabled())
            for _ in range(200):
                if v._rst_state == "done": break
                spin(0.02)
            spin(0.3); self.assertEqual(v._rst_state, "done"); self.assertTrue(v.btn_rst.isEnabled())
            calls = open(log).read().split("\n"); self.assertEqual(calls[0], "stop"); self.assertTrue(calls[1].startswith("start"))
            self.assertGreater(v.btn_rviz.height(), 24); self.assertEqual(v.btn_rst.text().split()[-1], "RST")
        finally:
            R.RvizMapView.MAP_SH = keep; w.close()


class TestModeTiles(unittest.TestCase):
    def test_tiles_click_active_disabled_and_compact(self):
        from nimbus_gcs.components.mode_tile import ModeTile
        t = ModeTile("ALT HOLD", "height", "alt"); t.resize(120, 72); hits = []
        t.clicked.connect(lambda: hits.append(1)); t.show(); t.click(); self.assertEqual(hits, [1])
        t.setText("● ALT HOLD"); self.assertEqual(t.label(), "ALT HOLD")
        t.set_active(True); spin(0.05); self.assertTrue(t._anim.isActive()); a = t.grab()
        t.setEnabled(False); spin(0.02); self.assertFalse(t._anim.isActive())                              # nothing animates when disabled
        t.set_compact(True); self.assertFalse(t.grab().isNull()); self.assertFalse(a.isNull())


class TestBatteryChip(unittest.TestCase):
    def test_states(self):
        from nimbus_gcs.components.battery_gauge import BatteryGauge
        g = BatteryGauge(); g.set_battery(None, None); self.assertEqual(g.value(), "--"); self.assertEqual(g.level(), "normal")
        g.set_battery(64, 15.6); self.assertEqual(g.value(), "64%"); self.assertEqual(g.level(), "normal")
        g.set_battery(22, 14.2); self.assertEqual(g.level(), "caution"); self.assertNotEqual(g.level(), "alarm")       # battery: a warning only
        g.set_battery(130, 17.0); self.assertEqual(g.value(), "100%"); self.assertFalse(g.grab().isNull())


class TestVideoStopNeverBlocks(unittest.TestCase):
    def test_a_stuck_connect_does_not_block_stop_or_write_into_the_next_run(self):
        from nimbus_gcs.video_source import FakeCapture, VideoSource
        import numpy as np
        gate = threading.Event()

        def slow(kind, target):                                     # a connect that hangs (as an unreachable RTSP host does)
            gate.wait(3.0); return FakeCapture(lambda: np.zeros((4, 4, 3), np.uint8) + 7)
        s = VideoSource(opener=slow, load=False); s.set_source("network", "rtsp://x/a"); s.start(); time.sleep(0.05)
        t0 = time.monotonic(); s.stop(); self.assertLess(time.monotonic() - t0, 0.5)                         # was up to 3 s on the GUI thread
        s._opener = lambda k, t: FakeCapture(lambda: (time.sleep(0.01), np.zeros((4, 4, 3), np.uint8) + 1)[1])
        s.set_source("network", "rtsp://x/b"); s.start(); gate.set(); time.sleep(0.4)
        f = s.latest(); self.assertIsNotNone(f); self.assertEqual(int(f[0, 0, 0]), 1)                         # the old run's frames (7) never show up
        s.stop()

    def test_same_source_is_not_restarted(self):
        from nimbus_gcs.video_source import VideoSource
        s = VideoSource(opener=lambda k, t: None, load=False); s.set_source("network", "rtsp://x/a"); calls = []
        s.stop = lambda: calls.append(1); s.set_source("network", "rtsp://x/a"); self.assertEqual(calls, [])


if __name__ == "__main__":
    unittest.main()
