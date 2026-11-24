"""Offscreen tests for nimbus_gcs.viz widgets."""
import os
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
from PyQt6.QtGui import QColor, QImage
from PyQt6.QtWidgets import QApplication, QCheckBox, QMessageBox, QWidget

from nimbus_gcs import theme
from nimbus_gcs.viz.bottom_strip import BottomStrip
from nimbus_gcs.viz.height_plot import HeightPlot
from nimbus_gcs.viz.preflight_list import PreflightList
from nimbus_gcs.viz.scene_view import SceneView

app = QApplication.instance() or QApplication([])
theme.load_fonts()
SIZES = [(1280, 720), (1920, 1080)]


def colours(img: QImage) -> int:
    seen = set()
    for y in range(0, img.height(), 3):
        for x in range(0, img.width(), 3):
            seen.add(img.pixel(x, y))
    return len(seen)


def grab(w, size):
    w.resize(*size); w.show(); return w.grab().toImage()


class VizTests(unittest.TestCase):
    def test_all_paint_non_blank(self):
        rows = [("a", "Link", "ready", "ok"), ("b", "Battery", "13.8 V", "caution")]
        for size in SIZES:
            for mk in (HeightPlot, SceneView, BottomStrip, PreflightList):
                w = mk()
                if mk is PreflightList:
                    w.set_checks(rows)
                s = (size[0] // 2, size[1] // 3) if mk is PreflightList else size
                if mk is BottomStrip:
                    s = (size[0], 130)
                self.assertGreater(colours(grab(w, s)), 4, f"{mk.__name__} blank at {size}")

    def test_plot_ring_buffer(self):
        p = HeightPlot(); p.set_window(10)
        for i in range(100):
            p.push(i, 0.5, i / 200, 0.0)
        pts = p.points()
        self.assertLessEqual(pts[0][0], 100); self.assertGreaterEqual(pts[0][0], 88); self.assertEqual(pts[-1][0], 99)
        self.assertEqual(p.view_range(), (89, 99))
        p.clear(); self.assertEqual(p.points(), [])
        p.load_series([0, 1, 2], [0, .5, .5], [0, .2, .4], [0, 0, 0]); self.assertEqual(len(p.points()), 3)
        self.assertGreater(colours(grab(p, (900, 260))), 4)

    def test_scene_with_and_without_frame(self):
        s = SceneView(); s.set_values(0.52, 47, 0.12)
        a = grab(s, (1280, 720)); self.assertFalse(s.has_frame())
        img = QImage(640, 360, QImage.Format.Format_RGB32); img.fill(QColor("#336699"))
        s.set_frame(img); b = grab(s, (1280, 720)); self.assertTrue(s.has_frame()); self.assertNotEqual(a, b)
        s.set_frame(None); self.assertFalse(s.has_frame())

    def test_preflight_rows_and_signal(self):
        l = PreflightList(); l.resize(376, 300)
        l.set_checks([("link", "Link and RC", "ready", "ok"), ("batt", "Battery", "13.8 V · warning only", "caution"), ("fc", "Flight controller", "no heartbeat", "blocking")])
        self.assertEqual(l.ready_count(), 2)
        got = []; l.row_clicked.connect(got.append)
        from PyQt6.QtTest import QTest
        from PyQt6.QtCore import Qt, QPoint
        l.show(); QTest.mouseClick(l, Qt.MouseButton.LeftButton, pos=QPoint(100, 32 + 40 + 20))
        self.assertEqual(got, ["batt"])
        img = grab(l, (376, 260)); self.assertGreater(colours(img), 6)

    def test_bottom_strip_caution(self):
        b = BottomStrip(); b.set_values({"HEIGHT": "0.52", "RANGE": "0.55"})                      # 2026-11-24: HEIGHT + RANGE only (LQ / battery live in the top bar)
        a = grab(b, (1280, 36)); b.set_caution("RANGE", True); c = grab(b, (1280, 36)); self.assertNotEqual(a, c)

    def test_no_forbidden_widgets(self):
        import nimbus_gcs.viz as v, pathlib
        for f in pathlib.Path(v.__file__).parent.glob("*.py"):
            src = f.read_text(); self.assertNotIn("QCheckBox", src); self.assertNotIn("QMessageBox", src)


if __name__ == "__main__":
    unittest.main()
