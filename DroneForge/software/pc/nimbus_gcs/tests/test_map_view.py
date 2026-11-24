"""RTAB-Map 2D sparse map in the GCS: defaults, the view, and the Video page switching to it when the PC's stream is live."""
import os
import tempfile
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ["DRONEFORGE_VIDEO_STATE"] = os.path.join(tempfile.mkdtemp(), "video.json")
os.environ["DRONEFORGE_DEPTH"] = os.path.join(tempfile.mkdtemp(), "depth.json")

import numpy as np
from PyQt6.QtWidgets import QApplication

from nimbus_core.map_wire import MapServer, WallSkeleton
from nimbus_gcs.main_window import apply_dark

_app = QApplication.instance() or QApplication([])
apply_dark(_app)


def _house():
    g = np.full((160, 200), -1, np.int8); g[20:140, 20:180] = 0
    for a, b, c, d in ((20, 24, 20, 180), (136, 140, 20, 180), (20, 140, 20, 24), (20, 140, 176, 180), (20, 90, 98, 102)):
        g[a:b, c:d] = 100
    return g


def _item(grid, t=1.0):
    return {"grid": grid, "res": 0.05, "ox": -5.0, "oy": -4.0, "t": t}


def test_video_page_shows_the_rtabmap_map_in_rviz_colours_from_the_window_map_client():
    """2026-11-23: the Video page shows ONE map, drawn as RViz draws it (rviz_map_view.py), fed by the window's map client (map_stream_node, tcp 5765)."""
    from nimbus_gcs.video_page import VideoPage
    from nimbus_gcs.rviz_map_view import RAW_LUT, THIN_LUT
    from nimbus_core.map_wire import MapClient
    srv = MapServer(port=0, host="127.0.0.1"); srv.start()
    raw = _house(); thin = WallSkeleton().update(raw, (-5.0, -4.0))
    srv.publish("map_raw", raw, 0.05, (-5.0, -4.0), (0.0, 0.0, 0.3)); srv.publish("map_thin", thin, 0.05, (-5.0, -4.0), (0.0, 0.0, 0.3))
    from nimbus_gcs.tests.test_pages_a import _win
    w = _win(); w.map_client = MapClient(); w.map_client.start("127.0.0.1", srv.port)
    p = VideoPage(w); p.resize(1600, 800); p.show()
    t0 = time.time()
    while time.time() - t0 < 4 and len(p.rvmap._imgs) < 2:
        _app.processEvents(); time.sleep(0.05)
    assert set(p.rvmap._imgs) == {"raw", "thin"}
    assert abs(p.view.width() - p.rvmap.width()) <= 2                               # half video, half map
    assert tuple(RAW_LUT[0xFF]) == (112, 137, 134, 255) and tuple(RAW_LUT[0]) == (255, 255, 255, 255) and tuple(THIN_LUT[100]) == (255, 0, 255, 255)   # RViz map / costmap palettes
    w.map_client.stop(); srv.stop(); p.close()
