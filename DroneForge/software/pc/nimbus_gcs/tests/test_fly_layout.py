"""Fly page: fits at every window size without scrolling or clipping, and shows the shared live video on the scene (offscreen)."""
import os
import tempfile
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("DRONEFORGE_HOLD_MS", "0")
os.environ.setdefault("DRONEFORGE_UI_STATE", os.path.join(tempfile.mkdtemp(), "ui.json"))
os.environ.setdefault("DRONEFORGE_VIDEO_STATE", os.path.join(tempfile.mkdtemp(), "video.json"))
import numpy as np
from PyQt6.QtWidgets import QApplication, QScrollArea, QWidget

from nimbus_gcs import theme
from nimbus_gcs.main_window import MainWindow

_app = QApplication.instance() or QApplication([])
theme.load_fonts(); _app.setStyleSheet(theme.qss())
SIZES = [(1280, 720), (1366, 768), (1600, 900), (1920, 1080), (2560, 1440)]


def _spin(n=8):
    for _ in range(n):
        _app.processEvents()


class FlyLayoutTest(unittest.TestCase):
    def setUp(self):
        self.w = MainWindow(); self.w.show(); self.w.build_all()

    def tearDown(self):
        self.w.model.disconnect(); self.w.close()

    def test_fits_without_scroll_or_clipping(self):
        w = self.w
        for W, H in SIZES:
            w.resize(W, H); w.goto("FLY"); _spin(); w.refresh(); _spin()
            f = w.fly
            self.assertIsNone(w.workspaces["FLY"].scroll_area(0), "the Fly page must not sit in a scroll area")
            self.assertFalse([s for s in f.window().findChildren(QScrollArea) if s.isVisible() and s.verticalScrollBar().isVisible() and s.isAncestorOf(f)])
            self.assertLessEqual(f.minimumSizeHint().height(), f.height(), f"{W}x{H}: page needs {f.minimumSizeHint().height()} > {f.height()}")
            self.assertLessEqual(f.sizeHint().height() if f.sizeHint().height() < 0 else 0, f.height())
            self.assertLessEqual(f.minimumSizeHint().width(), f.width())
            r = f.rect()
            for c in f.findChildren(QWidget):
                if c.isVisibleTo(f) and c.window() is f.window() and c.parent() is not None and not c.objectName().startswith("qt_"):
                    # the PiP lives inside the scene; everything else must lie inside the page
                    tl = c.mapTo(f, c.rect().topLeft()); br = c.mapTo(f, c.rect().bottomRight())
                    self.assertTrue(r.adjusted(-1, -1, 1, 1).contains(tl) and r.adjusted(-1, -1, 1, 1).contains(br), f"{W}x{H}: {type(c).__name__} {tl}-{br} outside {r}")
            # nothing overlaps in the right column
            col = [f.btn_main, f.btn_disarm, f.height_card, f.mode_card, f.preflight]
            ys = sorted((c.mapTo(f, c.rect().topLeft()).y(), c.mapTo(f, c.rect().bottomLeft()).y()) for c in col if c.isVisible())
            for (a0, a1), (b0, b1) in zip(ys, ys[1:]):
                self.assertLessEqual(a1, b0 + 1, f"{W}x{H}: right column overlaps")
            self.assertLessEqual(f.strip.mapTo(f, f.strip.rect().bottomLeft()).y(), f.height())

    def test_right_column_and_strip_fill_page(self):
        w = self.w
        for W, H in SIZES + [(1863 + 0, 950 + 56 + 0)]:
            w.resize(W, H); w.goto("FLY"); _spin(); w.refresh(); _spin()
            f = w.fly
            bot = lambda c: c.mapTo(f, c.rect().bottomLeft()).y()
            from nimbus_gcs.viz.bottom_strip import STRIP_H, STRIP_H_COMPACT
            self.assertEqual(f.strip.height(), STRIP_H_COMPACT if f._compact else STRIP_H)
            self.assertLessEqual(bot(f.strip) + 1, f.height(), f"{W}x{H}: strip clipped")
            self.assertLessEqual(f.strip.mapTo(w, f.strip.rect().bottomLeft()).y() + 1, w.height())
            self.assertLessEqual(abs(bot(f.preflight) - bot(f.strip)), 12, f"{W}x{H}: right column {bot(f.preflight)} vs strip {bot(f.strip)}")   # 2026-11-24: the pre-flight list runs to the bottom, level with the readout strip
            self.assertGreaterEqual(f.plot.height(), 140)
            self.assertFalse([c for c in f.findChildren(QScrollArea) if c.isVisible()])
            self.assertEqual(f._compact, f.height() < f.COMPACT_BELOW)

    def test_video_on_scene_and_fallback_tags(self):
        from nimbus_gcs.video_source import FakeCapture, VideoSource
        w = self.w; w.resize(1600, 900); w.goto("FLY"); _spin()
        frame = np.full((72, 128, 3), 90, np.uint8)
        src = VideoSource(opener=lambda k, t: FakeCapture(lambda: frame.copy()), load=False); src.set_source("network", "udp://x"); w.video_source = src
        f = w.fly
        f._video_tick(); self.assertFalse(f.scene.has_frame()); self.assertEqual(f.scene.video_tag(), "VIDEO OFF")
        src.start()
        import time
        t0 = time.time()
        while src.state != "live" and time.time() - t0 < 3:
            time.sleep(0.05)
        f._video_tick(); self.assertTrue(f.scene.has_frame()); self.assertEqual(f.scene.video_tag(), "")
        src.stop(); f._video_tick(); self.assertFalse(f.scene.has_frame()); self.assertEqual(f.scene.video_tag(), "VIDEO OFF")


if __name__ == "__main__":
    unittest.main()
