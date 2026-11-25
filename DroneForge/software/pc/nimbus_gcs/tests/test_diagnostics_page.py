"""DIAGNOSTICS tab: built, wired, and filled from real numbers (no hardware: a fake radio session / probe)."""
import os
import tempfile
import time
import unittest
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ["DRONEFORGE_SETTINGS"] = os.path.join(tempfile.mkdtemp(), "s.json"); os.environ["DRONEFORGE_FLIGHTS"] = tempfile.mkdtemp()
os.environ["DRONEFORGE_VIDEO_STATE"] = os.path.join(tempfile.mkdtemp(), "video.json")

from PyQt6.QtWidgets import QApplication, QCheckBox, QMessageBox

from nimbus_core import rc_check as RC
from nimbus_gcs import diagnostics_page as D
from nimbus_gcs import theme
from nimbus_gcs.main_window import MainWindow, apply_dark
from nimbus_gcs.video_source import VideoSource
from nimbus_pc import crsf

_app = QApplication.instance() or QApplication([])
theme.load_fonts(); apply_dark(_app)


class TestJudges(unittest.TestCase):
    def test_states_read_without_colour(self):
        self.assertEqual(D.judge_rate(100, 90, 50), D.OK); self.assertEqual(D.judge_rate(60, 90, 50), D.CAUTION); self.assertEqual(D.judge_rate(10, 90, 50), D.FAIL); self.assertEqual(D.judge_rate(None, 90, 50), D.NONE)
        self.assertEqual(D.judge_low(0.012 * 1000, 30, 249), D.OK); self.assertEqual(D.judge_low(120, 30, 249), D.CAUTION); self.assertEqual(D.judge_low(400, 30, 249), D.FAIL)
        for s in (D.OK, D.CAUTION, D.FAIL): self.assertTrue(s[0] in "✔⚠✖")

    def test_packet_rate_from_the_tx_timing_frame(self):
        iv = int(10000 * 10.0).to_bytes(4, "big")                                                       # 10 000 us = 100 Hz, in 0.1 us units
        st = SimpleNamespace(last_payload={0x3A: b"\x00\x00\x10" + iv + b"\x00\x00\x00\x00"})
        self.assertAlmostEqual(D.packet_hz(st), 100.0, places=3); self.assertIsNone(D.packet_hz(SimpleNamespace(last_payload={})))


class TestPage(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        MainWindow.video_source = VideoSource(opener=lambda k, t: None, load=False)          # a stand-in source: no network, no camera thread
        cls.w = MainWindow(); cls.w.resize(1366, 768); cls.w.show()

    @classmethod
    def tearDownClass(cls):
        cls.w.model.disconnect(); cls.w.close(); cls.w.deleteLater(); _app.processEvents(); cls.w = None; del MainWindow.video_source      # delete the window before the application goes (else Qt crashes at exit)

    def test_the_tab_exists_and_has_no_dialogs_or_check_boxes(self):
        w = self.w; self.assertIn("DIAGNOSTICS", w.WORKSPACES); self.assertEqual(w.tabs._labels[-2], "DIAGNOSTICS")
        w.goto("DIAGNOSTICS"); _app.processEvents(); p = w.diagnostics_page
        self.assertEqual(p.findChildren(QCheckBox), []); self.assertEqual((p.tbl.rowCount(), p.rates.rowCount()), (16, 9))
        self.assertEqual([p.tbl.item(i, 1).text() for i in (4, 8, 12, 13, 14)], ["ARM", "HEIGHT", "AID FWD", "AID RIGHT", "AID VALID"])
        self.assertTrue(w.sc_nums[5].key().toString() == "6")                                          # key 6 = the last workspace (SETTINGS), 5 = DIAGNOSTICS

    def test_offline_it_says_so_and_the_test_button_is_off(self):
        w = self.w; w.goto("DIAGNOSTICS"); p = w.diagnostics_page; p.refresh({}); _app.processEvents()
        self.assertFalse(p.btn_test.isEnabled()); self.assertEqual(p.rates.item(0, 3).text(), D.NONE)
        p.run_test(); self.assertIn("live radio link", p.test_line.text())

    def test_it_shows_sent_received_diff_test_and_rates_from_a_fake_live_link(self):
        w = self.w; w.goto("DIAGNOSTICS"); p = w.diagnostics_page; core = w.model.core; st = w.model.state
        pat = [1500] * 16
        for ch, us in RC.PATTERN.items(): pat[ch - 1] = us
        sender = SimpleNamespace(last_channels=[RC.us_to_ticks(u) for u in pat], frames_sent=500, recent_max_gap_s=0.012, stale_trips=0, max_interval_s=0.02, _gaps=[])
        probe = SimpleNamespace(rc=[v + (1 if i == 12 else 0) for i, v in enumerate(pat)], t_rc=time.monotonic(), rc_reply_hz=1.6, rc_rtt_s=0.45, rc_req_t=time.monotonic(), replies=9)
        saved = (core.sess, core.probe, core.state_source)
        try:
            core.sess = SimpleNamespace(sender=sender); core.probe = probe; core.state_source = "live"
            for _ in range(100): st.out_times.setdefault(crsf.TYPE_RC_CHANNELS, __import__("collections").deque(maxlen=400)).append(st.clock())
            core.rc_check.rows = RC.judge(probe.rc)[2]; core.rc_check.state = "ok"; core.rc_check.detail = "CH9-16 reach the FC intact (largest error 1 us)"
            p.refresh({})
            self.assertEqual(p.tbl.item(12, 2).text(), "1537 us"); self.assertEqual(p.tbl.item(12, 3).text(), "1538 us"); self.assertEqual(p.tbl.item(12, 4).text(), "+1")
            self.assertIn("✔", p.tbl.item(12, 6).text()); self.assertEqual(p.tbl.item(0, 6).text(), "--")                  # CH1-8 are not part of the pattern test
            self.assertEqual(p.rates.item(1, 1).text(), "12 ms"); self.assertEqual(p.rates.item(1, 3).text(), D.OK); self.assertEqual(p.rates.item(2, 3).text(), D.OK)
            self.assertEqual(p.rates.item(7, 3).text(), D.OK); self.assertIn("1.6 Hz", p.rates.item(7, 1).text())
            self.assertIn("Channel test: CH9-16 reach the FC intact", p.test_line.text())
            core.rc_check.state = "fail"; core.rc_check.detail = "CH13-16 are NOT carried"; p.refresh({}); self.assertIn("✖", p.test_line.text())
            sender.stale_trips = 5; sender.recent_max_gap_s = 0.4; p.refresh({}); self.assertEqual(p.rates.item(2, 3).text(), D.FAIL); self.assertEqual(p.rates.item(1, 3).text(), D.FAIL)
        finally:
            core.sess, core.probe, core.state_source = saved; core.rc_check.reset()

    def test_preflight_lists_the_channel_row(self):
        w = self.w; ids = {c.id for c in w.model.core.director.preflight()}; self.assertIn("rc_channels", ids)


if __name__ == "__main__":
    unittest.main()
