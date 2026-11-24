"""Optional PNG plot of a hover run (needs PyQt6; uses the GCS chart widget). QT_QPA_PLATFORM=offscreen works without a display."""
from __future__ import annotations

import os
import sys


def save_png(res, path: str, hover_height: float = 0.5) -> None:
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PyQt6.QtGui import QColor
    from PyQt6.QtWidgets import QApplication, QVBoxLayout, QWidget
    from nimbus_gcs.widgets import StripChart
    app = QApplication.instance() or QApplication(sys.argv[:1])
    rows = res.rows
    series = lambda key, f=lambda v: v: [(r["t"], f(r[key])) for r in rows]
    w = QWidget(); lay = QVBoxLayout(w); w.resize(900, 760)
    c1 = StripChart(f"{res.name}: altitude (m)", [("z", "true", QColor("#3fb950")), ("z_est", "estimate", QColor("#4aa3ff")), ("ref", "target", QColor("#d29922"))], " m", -0.05, 1.3, window_s=len(rows) / 10 + 1)
    c2 = StripChart("horizontal position (m)", [("x", "x fwd", QColor("#4aa3ff")), ("y", "y right", QColor("#d29922"))], " m", None, None, window_s=len(rows) / 10 + 1)
    c3 = StripChart("throttle and tilt", [("thr", "throttle", QColor("#3fb950")), ("tilt", "tilt/10 deg", QColor("#f85149"))], "", 0.0, 0.8, window_s=len(rows) / 10 + 1)
    c1.set_data({"z": series("z"), "z_est": series("z_est"), "ref": [(r["t"], hover_height if r["state"] in ("hover", "sign_check") else float("nan")) for r in rows]})
    c2.set_data({"x": series("x"), "y": series("y")})
    c3.set_data({"thr": series("thr"), "tilt": series("tilt_deg", lambda v: v / 10.0)})
    for c in (c1, c2, c3):
        lay.addWidget(c)
    from PyQt6.QtCore import QTimer
    w.show(); app.processEvents()
    w.grab().save(path)
    w.close()
