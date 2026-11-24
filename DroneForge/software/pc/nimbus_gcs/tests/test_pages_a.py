"""Restyled VIDEO / ANALYZE pages: build offscreen, no dialogs / check boxes, theme tokens only, no ad-hoc hex colours."""
import os
import re
import tempfile
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ["DRONEFORGE_VIDEO_STATE"] = os.path.join(tempfile.mkdtemp(), "video.json")

import pytest
from PyQt6.QtWidgets import QApplication, QCheckBox, QMessageBox

from nimbus_gcs import theme
from nimbus_gcs.video_source import VideoSource


@pytest.fixture(scope="module")
def app():
    a = QApplication.instance() or QApplication([]); theme.load_fonts(); a.setStyleSheet(theme.qss()); return a


def _win():
    model = SimpleNamespace(connected=False, core=SimpleNamespace(sess=None, autonomy=SimpleNamespace(phase="idle")), state=SimpleNamespace(decoded={}), history={})
    return SimpleNamespace(model=model, video_source=VideoSource(opener=lambda k, t: None, load=False))


def test_video_page_is_dji_only_with_feed_and_map(app):
    from nimbus_gcs.video_page import VideoPage
    from nimbus_gcs.components import Card
    p = VideoPage(_win()); p.resize(1920, 900)
    assert p.findChildren(QCheckBox) == [] and len(p.findChildren(Card)) >= 1
    assert p.url.text().strip() != "" and not hasattr(p, "seg") and not hasattr(p, "band") and not hasattr(p, "device")          # a stream address field; no USB / analog source, no VTX card
    assert [b.text() for b in (p.btn_start, p.btn_stop, p.btn_snap, p.btn_lat, p.btn_map, p.btn_map_stop, p.btn_map_clear)] == ["Start", "Stop", "Snapshot", "Latency test", "Start map", "Stop map", "Clear map"]


def test_analysis_view_cards_and_levels(app):
    from nimbus_gcs.analysis_page import AnalysisView, LEVEL
    v = AnalysisView(); assert len(v.charts) == 4 and v.summary_card.title() == "FINDINGS"
    v._text("x", [("warn", "a"), ("bad", "b"), ("ok", "c")], False)
    assert theme.AMB in v.findings.text() and theme.RED in v.findings.text()
    assert {sym for sym, _ in LEVEL.values()} == {"✔", "⚠", "✖"}      # state readable without colour


def test_no_adhoc_colours_or_dialogs():
    base = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    for f in ("video_page.py", "video_pip.py", "logs_tab.py", "analysis_page.py", "rtsp_direct.py", "telemetry_page.py", "live_page.py", "flights_page.py", "video_health.py", "video_latency.py", "video_source.py"):
        src = open(os.path.join(base, f)).read()
        assert not re.search(r"#[0-9a-fA-F]{6}\b", src), f
        assert "QMessageBox" not in src and "QCheckBox" not in src, f
