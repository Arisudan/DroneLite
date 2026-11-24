import os
import time
import unittest

import tempfile                                  # noqa: E402

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("DRONEFORGE_HOLD_MS", "0")
os.environ["DRONEFORGE_UI_STATE"] = os.path.join(tempfile.mkdtemp(prefix="df_ui_"), "gcs_ui.json")      # never touch the real ~/.config file

from PyQt6.QtTest import QTest                       # noqa: E402
from PyQt6.QtCore import Qt                          # noqa: E402
from PyQt6.QtWidgets import QApplication             # noqa: E402

from nimbus_gcs.main_window import MainWindow, apply_dark   # noqa: E402
from nimbus_gcs.model import GcsModel                       # noqa: E402
from nimbus_gcs.widgets import AttitudeWidget, ChannelBars, StripChart  # noqa: E402
from nimbus_pc.emulator import DongleEmulator               # noqa: E402

_app = QApplication.instance() or QApplication([])
apply_dark(_app)


class TestModel(unittest.TestCase):
    def setUp(self):
        self.emu = DongleEmulator().start()
        self.m = GcsModel()
        self.m.connect(self.emu.path)

    def tearDown(self):
        self.m.disconnect(); self.emu.stop()

    def test_connect_status_and_events(self):
        for _ in range(40):
            self.m.tick(); time.sleep(0.03)
        s = self.m.status()
        self.assertTrue(s["connected"]); self.assertTrue(s["sender_alive"])
        self.assertIsNotNone(s["lq"])
        self.assertTrue(any("connected to" in t for _, t in self.m.state.events))

    def test_sweep_never_arms_and_safe_button_resets(self):
        self.m.set_source("sweep"); self.m.set_allow_arm(True)
        for _ in range(30):
            self.m.tick(); time.sleep(0.02)
        self.assertFalse(self.m.last_controls.arm)                     # sweep never arms, even with arming allowed
        self.m.safe_now()
        self.assertEqual(self.m.source, "idle"); self.assertFalse(self.m.allow_arm)
        self.assertFalse(self.m.sess.sender.allow_arm)
        self.assertEqual(self.emu.last_channels[4] if self.emu.last_channels else 172, 172)   # arm channel low at the dongle

    def test_history_is_sampled_and_bounded(self):
        for _ in range(30):
            self.m.sample(); time.sleep(0.05)
        self.assertGreater(len(self.m.history["up_lq"]), 3)
        self.assertGreater(len(self.m.history["battery_v"]), 3)


class TestWarnings(unittest.TestCase):
    base = {"connected": True, "no_data_s": 0.2, "sender_alive": True, "watchdog": "ok", "crc_errors": 0}

    def test_safety_banners(self):
        from nimbus_gcs.model import warnings
        txt = " | ".join(warnings({**self.base, "safety_disarm": "tilt_extreme", "safety_land": "battery"}, 100, 10))
        self.assertIn("SAFETY DISARM (tilt_extreme)", txt); self.assertIn("LAND requested (battery)", txt)

    def test_each_banner(self):
        from nimbus_gcs.model import warnings
        self.assertEqual(warnings(self.base, 1000, 60), [])
        self.assertIn("NO DATA", warnings({**self.base, "no_data_s": 9}, 10, 60)[0])
        self.assertIn("SENDER STOPPED", warnings({**self.base, "sender_alive": False}, 10, 60)[0])
        self.assertIn("WATCHDOG TRIPPED (missing-frames)", warnings({**self.base, "watchdog": "missing-frames"}, 10, 60)[0])
        self.assertIn("SERIAL ERRORS", warnings({**self.base, "crc_errors": 32}, 58, 60)[0])
        self.assertIn("TX NOT FORWARDING", warnings({**self.base, "tx_stuck": True}, 100, 60)[0])
        self.assertEqual(warnings({**self.base, "crc_errors": 32}, 5000, 60), [])        # 32 stray errors in 5000 frames is noise
        self.assertEqual(warnings({**self.base, "connected": False, "no_data_s": 99}, 0, 60), [])


class TestArmingChecklist(unittest.TestCase):
    def test_checklist_explains_what_is_missing_and_goes_green(self):
        emu = DongleEmulator().start(); m = GcsModel()
        try:
            m.connect(emu.path)
            items = {n: ok for n, ok, _ in m.arming_status()}
            self.assertTrue(items["Connected to the dongle"]); self.assertFalse(items["Arming allowed"])
            self.assertFalse([n for n in items if "Boxer" in n or "gamepad" in n.lower() or "SA)" in n or "SC switch" in n], "no gamepad or Boxer rows")
            self.assertIn("Arming allowed", m.arming_summary())
            m.set_allow_arm(True)
            self.assertTrue({n: ok for n, ok, _ in m.arming_status()}["Arming allowed"])
            self.assertTrue(m.arming_summary().startswith("Ready"))
        finally:
            m.disconnect(); emu.stop()


class TestWidgets(unittest.TestCase):
    def test_paint_with_empty_and_bad_data(self):
        c = StripChart("t", [("a", "a", __import__("PyQt6.QtGui", fromlist=["QColor"]).QColor("#fff"))])
        c.resize(300, 120); c.grab()
        c.set_data({"a": [(0.0, float("nan")), (1.0, 2.0), (2.0, 3.0)]}); c.grab()
        a = AttitudeWidget(); a.resize(200, 200); a.set_attitude(None, None, None); a.grab(); a.set_attitude(10.0, -20.0, 5.0); a.grab()
        b = ChannelBars(); b.set_values([988, 1500, 2012, 1500, 2000, 988, 988, 1500]); b.grab()


class TestWindow(unittest.TestCase):
    def setUp(self):
        self.emu = DongleEmulator().start()
        self.win = MainWindow(); self.win.port_box.setCurrentText(self.emu.path); self.win.show()
        self.win.build_all()

    def tearDown(self):
        self.win.close(); self.emu.stop()

    def test_window_shows_live_values(self):
        self.win.toggle_connection()
        QTest.qWait(2500)
        tp = self.win.telemetry_page
        names = {tp.tel.item(r, 0).text(): tp.tel.item(r, 1).text() for r in range(tp.tel.rowCount())}
        self.assertEqual(names["droneLinked"], "linked")
        self.assertIn("up LQ", names["linkStats"])
        self.assertRegex(names["optrange"], r"dist \d+\.\d+ m")
        self.assertEqual(self.win.btn_connect.text(), "Disconnect")
        self.win.goto("ANALYZE", "Live data"); QTest.qWait(300)
        self.assertIn("ExpressLRS TX", self.win.radio_page.devices.text())
        self.assertFalse(self.win.banner.isVisible())
        img = self.win.grab(); self.assertGreater(img.width(), 800)

    def test_no_data_banner_when_telemetry_stops(self):
        self.win.toggle_connection(); QTest.qWait(1200)
        self.emu.telemetry_enabled = False
        QTest.qWait(4500)
        self.assertTrue(self.win.banner.isVisible())
        self.assertIn("NO DATA", self.win.banner.text())




class TestFlightView(unittest.TestCase):
    def test_flight_view_live_branch_with_attitude_and_sim_branch(self):
        emu = DongleEmulator().start(); m = GcsModel()
        try:
            m.connect(emu.path)
            for _ in range(60):
                m.tick(); time.sleep(0.03)
            fv = m.flight_view()                                   # live: height estimate only, no position
            self.assertIsNone(fv["x"]); self.assertFalse(fv["truth"]); self.assertIn("pitch", fv)
            m.disconnect()
            m.connect_sim(); time.sleep(1.0)
            fv = m.flight_view()
            self.assertTrue(fv["truth"]); self.assertEqual(fv["x"], 0.0)
        finally:
            m.disconnect(); emu.stop()


EXPECTED = {"FLY": ["Fly"], "VIDEO": ["Video"], "SETUP": ["Setup"],
            "ANALYZE": ["Flights", "Live data"], "SETTINGS": ["Settings"]}


class TestNavigationAndLayout(unittest.TestCase):
    def setUp(self):
        from nimbus_gcs.workspace import state_path
        try:
            os.remove(state_path())
        except OSError:
            pass

    def test_five_workspaces_with_expected_subtabs_every_page_fits(self):
        from PyQt6.QtWidgets import QScrollArea
        w = MainWindow()
        try:
            self.assertEqual(w.tabs._labels, list(EXPECTED))
            w.resize(1366, 740); w.show(); _app.processEvents()                            # the smallest supported window
            for name, titles in EXPECTED.items():
                ws = w.workspaces[name]
                self.assertEqual(ws.titles(), titles)
                for t in titles:
                    self.assertTrue(w.goto(name, t)); _app.processEvents()
                    self.assertEqual(ws.current_title(), t); self.assertEqual(w.current_workspace(), name)
                    self.assertEqual(w.tabs.current(), list(EXPECTED).index(name))
                    self.assertEqual(ws.bar.isVisible(), len(titles) > 1)
                    sa = ws.scroll_area(ws.current_index())
                    if name == "FLY":
                        self.assertIsNone(sa); continue                 # the Fly page is elastic and never scrolls
                    self.assertIsInstance(sa, QScrollArea)
                    self.assertLessEqual(sa.widget().minimumSizeHint().width(), 1100, f"{name}/{t} is too wide for the smallest window")
                    self.assertFalse(sa.horizontalScrollBar().isVisible(), f"{name}/{t} scrolls horizontally")
            self.assertFalse(w.goto("NOPE")); self.assertFalse(w.goto("SETUP", "Nope"))
        finally:
            w.model.disconnect(); w.close()

    def test_persistence_round_trip(self):
        import json
        from nimbus_gcs.workspace import state_path
        w = MainWindow(); w.goto("SETUP", "Radio"); w.goto("ANALYZE", "Live data"); w.close()
        d = json.load(open(state_path(), encoding="utf-8")); self.assertEqual(d["workspace"], "ANALYZE"); self.assertEqual(d["subtabs"]["SETUP"], "Setup")
        w2 = MainWindow()
        try:
            self.assertEqual(w2.current_workspace(), "FLY"); self.assertEqual(w2.workspaces["ANALYZE"].current_title(), "Live data")      # always opens on FLY; sub-tabs remembered
            self.assertEqual(w2.workspaces["SETUP"].current_title(), "Setup")
        finally:
            w2.model.disconnect(); w2.close()
        open(state_path(), "w").write("{not json")                                         # a damaged file is ignored
        w3 = MainWindow(); self.assertEqual(w3.current_workspace(), "FLY"); w3.model.disconnect(); w3.close()

    def test_keys_and_say(self):
        w = MainWindow(); w.show(); _app.processEvents()
        try:
            w.sc_nums[2].activated.emit(); self.assertEqual(w.current_workspace(), "SETUP")
            w.sc_ws_next.activated.emit(); self.assertEqual(w.current_workspace(), "ANALYZE")
            w.sc_ws_prev.activated.emit(); w.sc_ws_prev.activated.emit(); self.assertEqual(w.current_workspace(), "VIDEO")
            w.goto("ANALYZE", "Flights"); w.sc_sub_next.activated.emit(); self.assertEqual(w.workspaces["ANALYZE"].current_title(), "Live data")
            w.sc_sub_prev.activated.emit(); w.sc_sub_prev.activated.emit(); self.assertEqual(w.workspaces["ANALYZE"].current_title(), "Live data")
            w.goto("SETUP"); w.port_box.lineEdit().setFocus(); _app.processEvents()
            w.sc_nums[0].activated.emit(); self.assertEqual(w.current_workspace(), "SETUP")          # typing in the port box does not switch workspace
            w.say("hello", "warn"); self.assertEqual(w.status_line.text(), "hello"); w._clear_say(); self.assertEqual(w.status_line.text(), "")
        finally:
            w.model.disconnect(); w.close()

    def test_preflight_rows_jump_to_the_fixing_page(self):
        w = MainWindow(); w.show()
        try:
            w.refresh(); f = w.fly
            ids = {c.id: c for c in f._checks}
            self.assertFalse(ids["range"].ok)
            f.preflight.row_clicked.emit("range"); self.assertEqual((w.current_workspace(), w.setup_page.nav.current()), ("SETUP", "sensors"))
            w.goto("FLY"); f.preflight.row_clicked.emit("link"); self.assertEqual(w.setup_page.nav.current(), "radio")
            w.goto("FLY"); f.preflight.row_clicked.emit("disarmed"); self.assertEqual(w.current_workspace(), "FLY")      # no goto: stays
        finally:
            w.model.disconnect(); w.close()

    def test_fly_page_fits_without_vertical_scroll(self):
        w = MainWindow(); w.resize(1920, 1080); w.show(); _app.processEvents(); w.refresh(); _app.processEvents()
        try:
            self.assertIsNone(w.workspaces["FLY"].scroll_area(0), "the Fly page must not be wrapped in a scroll area")
            self.assertLessEqual(w.fly.minimumSizeHint().height(), w.fly.height(), "the Fly page does not fit at 1920x1080")
            for b in (w.fly.btn_main, w.fly.btn_disarm, w.fly.btn_land, w.fly.scene, w.fly.phase_lbl, w.fly.plot, w.fly.preflight, w.fly.strip, w.fly.height_field):
                self.assertTrue(b.isVisible())
        finally:
            w.model.disconnect(); w.close()

class TestBatteryPercent(unittest.TestCase):
    def test_curve_monotonic_and_known_points(self):
        from nimbus_gcs.model import battery_percent
        self.assertEqual(battery_percent(16.8), 100.0); self.assertEqual(battery_percent(13.0), 0.0)
        self.assertAlmostEqual(battery_percent(15.2), 50.0, delta=1.0)             # 3.80 V/cell
        vals = [battery_percent(v / 10) for v in range(130, 175)]
        self.assertEqual(vals, sorted(vals))

    def test_live_percentage_in_status_led_and_summary(self):
        emu = DongleEmulator(); emu.battery_dv = 152; emu.start(); w = MainWindow()
        try:
            w.model.connect(emu.path)
            end = time.time() + 6
            while time.time() < end:
                _app.processEvents(); time.sleep(0.02)
            s = w.model.status()
            self.assertAlmostEqual(s["battery_pct"], 50.0, delta=3.0); self.assertAlmostEqual(s["battery_cell_v"], 3.8, delta=0.02)
            w.build_all(); w.refresh()
            self.assertIn("V/cell", w.led["Battery"].detail)
            w.model.disconnect(); w.refresh()
            self.assertEqual(w.led["Battery"].detail, "")
        finally:
            w.model.disconnect(); w.close(); emu.stop()


class TestSensorTab(unittest.TestCase):
    def _series(self, f):
        return [(i * 0.1, f(i * 0.1)) for i in range(80)]

    def test_fusion_report_recognises_a_filter_that_follows_and_one_that_does_not(self):
        from nimbus_gcs.sensor_tab import fusion_report, OK, BAD, INFO
        import math
        rng = self._series(lambda t: 0.5 + 0.3 * math.sin(t))                       # lifted up and down by hand
        good = {"fz_range": rng, "fz_ekf": [(t, v + 0.01) for t, v in rng], "fvz_ekf": self._series(lambda t: 0.3 * math.cos(t)),
                "fv_flow": self._series(lambda t: 0.3 + 0.2 * math.sin(2 * t)), "fv_ekf": self._series(lambda t: 0.3 + 0.2 * math.sin(2 * t) + 0.02)}
        lv = [l for l, _ in fusion_report(good)]
        self.assertEqual(lv, [OK, OK, OK])
        bad = dict(good); bad["fz_ekf"] = [(t, 0.9) for t, _ in rng]; bad["fvz_ekf"] = self._series(lambda t: 0.0); bad["fv_ekf"] = self._series(lambda t: 0.0)
        self.assertEqual([l for l, _ in fusion_report(bad)][0], BAD)                  # EKF height stuck 0.4 m away from the sensor
        self.assertIn(BAD, [l for l, _ in fusion_report(bad)][1:])                    # EKF speed does not follow the flow
        still = {"fz_range": self._series(lambda t: 0.5), "fz_ekf": self._series(lambda t: 0.5), "fvz_ekf": self._series(lambda t: 0.0),
                 "fv_flow": self._series(lambda t: 0.0), "fv_ekf": self._series(lambda t: 0.0)}
        self.assertEqual([l for l, _ in fusion_report(still)][1:], [INFO, INFO])      # nothing moved: it says to move the drone, it does not guess
        self.assertEqual(fusion_report({})[0][0], INFO)

    def test_tab_fills_from_decoded_telemetry(self):
        w = MainWindow()
        try:
            st = w.model.state
            st.decoded["optrange"] = {"distance_m": 0.5, "dist_strength": 120, "dist_precision": 1, "flow_quality": 98, "flow_status": 1, "velocity_forward": 0.1, "velocity_right": -0.05}
            st.decoded["kinematic"] = {"valid": True, "x_m": 0.0, "y_m": 0.0, "z_m": 0.5, "vx_mps": 0.1, "vy_mps": 0.0, "vz_mps": 0.0}
            st.decoded_t["optrange"] = st.decoded_t["kinematic"] = st.clock()
            w.sensor_tab.refresh()
            self.assertIn("flow quality: 98", w.sensor_tab.sensor.text()); self.assertIn("EKF state valid: yes", w.sensor_tab.ekf.text())
        finally:
            w.model.disconnect(); w.close()


class TestFcLogs(unittest.TestCase):
    """Rules of the Logs page, on a folder that stands in for the mounted SD card."""

    def setUp(self):
        import tempfile
        self.tmp = tempfile.mkdtemp(); self.card = os.path.join(self.tmp, "card"); os.makedirs(self.card)
        self.dest = os.path.join(self.tmp, "pc"); self.ledger = os.path.join(self.tmp, "ledger.json")
        for n, data in (("LOG00001.BFL", b"a" * 3000), ("LOG00002.BFL", b""), ("LOG00003.BFL", b"c" * 5000)):
            open(os.path.join(self.card, n), "wb").write(data)
        open(os.path.join(self.card, "notes.docx"), "wb").write(b"not a log")

    def test_only_log_files_are_listed_and_nothing_is_deletable_before_download(self):
        from nimbus_gcs import fc_logs as FL
        logs = FL.list_logs(self.card)
        self.assertEqual([l.name for l in logs].count("notes.docx"), 0); self.assertEqual(len(logs), 3)
        by = {l.name: l for l in logs}
        self.assertFalse(FL.can_delete(by["LOG00001.BFL"], self.ledger)[0])               # never downloaded
        self.assertTrue(FL.can_delete(by["LOG00002.BFL"], self.ledger)[0])                # empty: nothing to lose
        ok, why = FL.delete(by["LOG00001.BFL"], self.card, self.ledger)
        self.assertFalse(ok); self.assertTrue(os.path.exists(by["LOG00001.BFL"].path))   # still on the card

    def test_download_never_overwrites_an_older_log_with_the_same_name(self):
        """the card restarts its numbering after it is cleared: a new LOG00001 must not replace an older saved LOG00001"""
        from nimbus_gcs import fc_logs as FL
        by = {l.name: l for l in FL.list_logs(self.card)}; f = by["LOG00001.BFL"]
        ok1, d1 = FL.download(f, self.dest, self.ledger); self.assertTrue(ok1)
        old = open(d1, "rb").read()
        ok2, d2 = FL.download(f, self.dest, self.ledger); self.assertTrue(ok2)
        self.assertNotEqual(d1, d2); self.assertEqual(open(d1, "rb").read(), old); self.assertTrue(os.path.exists(d2))

    def test_download_verifies_then_delete_removes_only_that_file(self):
        from nimbus_gcs import fc_logs as FL
        by = {l.name: l for l in FL.list_logs(self.card)}; f = by["LOG00001.BFL"]
        ok, dst = FL.download(f, self.dest, self.ledger); self.assertTrue(ok); self.assertEqual(open(dst, "rb").read(), b"a" * 3000)
        self.assertTrue(FL.can_delete(f, self.ledger)[0])
        ok, why = FL.delete(f, self.card, self.ledger); self.assertTrue(ok)
        self.assertFalse(os.path.exists(f.path)); self.assertTrue(os.path.exists(by["LOG00003.BFL"].path))   # the other log is untouched

    def test_changed_card_file_or_damaged_copy_blocks_delete(self):
        from nimbus_gcs import fc_logs as FL
        f = {l.name: l for l in FL.list_logs(self.card)}["LOG00003.BFL"]
        ok, dst = FL.download(f, self.dest, self.ledger); self.assertTrue(ok)
        open(dst, "ab").write(b"x")                                                        # the saved copy got damaged
        self.assertFalse(FL.delete(f, self.card, self.ledger)[0]); self.assertTrue(os.path.exists(f.path))

    def test_delete_refuses_a_file_outside_the_card(self):
        from nimbus_gcs import fc_logs as FL
        outside = os.path.join(self.tmp, "outside.BFL"); open(outside, "wb").write(b"")
        lf = FL.LogFile(outside, "outside.BFL", 0, 0.0)
        ok, why = FL.delete(lf, self.card, self.ledger); self.assertFalse(ok); self.assertTrue(os.path.exists(outside))

    def test_page_exists_and_enter_is_refused_without_an_fc(self):
        from nimbus_gcs import fc_logs as FL
        old = FL.fc_serial_port; FL.fc_serial_port = lambda: None
        w = MainWindow()
        try:
            w.logs_tab.refresh(); self.assertFalse(w.logs_tab.btn_enter.isEnabled()); self.assertFalse(w.logs_tab.btn_del.isEnabled())
            self.assertIn("No FC on USB", w.logs_tab.lbl.text())
        finally:
            FL.fc_serial_port = old; w.model.disconnect(); w.close()

    def _no_boxes(self):
        from PyQt6.QtWidgets import QMessageBox
        def boom(*a, **k): raise AssertionError("a QMessageBox was created")
        for n in ("question", "information", "warning", "critical"):
            setattr(QMessageBox, n, staticmethod(boom))

    def test_no_dialog_when_pressing_buttons_offline(self):
        from PyQt6.QtWidgets import QMessageBox
        from nimbus_gcs import fc_logs as FL
        saved = {n: QMessageBox.__dict__[n] for n in ("question", "information", "warning", "critical")}
        old = FL.fc_serial_port; FL.fc_serial_port = lambda: None
        w = MainWindow()
        try:
            self._no_boxes()
            t = w.logs_tab
            t.on_enter(); self.assertIn("No FC", t.msg.text())
            t.on_delete(); t.on_download(False); t.on_analyze()
        finally:
            FL.fc_serial_port = old
            for n, v in saved.items(): setattr(QMessageBox, n, v)
            w.model.disconnect(); w.close()


class TestBfl(unittest.TestCase):
    """The log decoder and its plain-text summary (on the real logs saved on this PC when they are present)."""

    def _log(self, name):
        from nimbus_gcs import fc_logs as FL
        import glob
        arch = os.path.join(FL.ROOT, "archive", "flight_logs_2026-11-04_to_2026-11-12", "fc_blackbox")      # the 2026-11-06 logs (v1 burst / v3 ramp); card numbers repeat on later days
        hit = glob.glob(os.path.join(arch, "20261106", name + "_*.BFL")) or glob.glob(os.path.join(FL.DEST_ROOT, "20261106", name + "_*.BFL"))
        if not hit:
            self.skipTest(f"{name} is not saved on this PC")
        from nimbus_gcs import bfl
        lg = bfl.Log(hit[0]); lg.decode(); return bfl, lg

    def test_the_burst_log_is_flagged_and_the_ramped_log_is_not(self):
        bfl, bad = self._log("LOG00019")
        self.assertEqual(bad.corrupt, 0); self.assertIn("A THROTTLE STEP / BURST", bfl.summarize(bad))                 # v1 firmware: all motors idle -> 100 % in one frame
        bfl, ok = self._log("LOG00020")
        self.assertEqual(ok.corrupt, 0); self.assertIn("(smooth)", bfl.summarize(ok)); self.assertNotIn("BURST", bfl.summarize(ok))   # v3: ramp

    def test_decoder_survives_garbage(self):
        import tempfile
        from nimbus_gcs import bfl
        p = os.path.join(tempfile.mkdtemp(), "x.BFL"); open(p, "wb").write(b"H Field I name:a,b\nH Field I signed:0,0\nH Field I predictor:0,0\nH Field I encoding:1,1\nH Field P predictor:1,1\nH Field P encoding:0,0\nIgarbage\xff\xfe\x00zz")
        lg = bfl.Log(p); lg.decode(); self.assertIsInstance(bfl.summarize(lg), str)

    def test_page_has_analysis_controls(self):
        w = MainWindow()
        try:
            self.assertTrue(w.logs_tab.btn_file.isEnabled()); self.assertTrue(hasattr(w.logs_tab.report, "show_log"))
            w.logs_tab._analyze("/nonexistent/file.BFL", 10)
        finally:
            w.model.disconnect(); w.close()

    def test_analyze_findings_on_real_logs(self):
        bfl, bad = self._log("LOG00019"); r = bfl.analyze(bad)
        self.assertTrue(any(lv == "bad" and "burst" in tx.lower() for lv, tx in r["findings"]))
        bfl, ok = self._log("LOG00020"); r = bfl.analyze(ok)
        self.assertFalse(any("burst" in tx.lower() and lv == "bad" for lv, tx in r["findings"]))
        for rr in (r, bfl.analyze(bad)):
            self.assertLessEqual(len(rr["t"]), 500)
            for k, v in rr["series"].items(): self.assertEqual(len(v), len(rr["t"]), k)
            for lv, tx in rr["findings"]: self.assertIn(lv, ("ok", "warn", "bad")); self.assertIsInstance(tx, str)

    def test_analyze_garbage_and_view(self):
        import tempfile
        from nimbus_gcs import bfl
        from nimbus_gcs.analysis_page import AnalysisView
        p = os.path.join(tempfile.mkdtemp(), "x.BFL"); open(p, "wb").write(b"H Field I name:a,b\nH Field I signed:0,0\nH Field I predictor:0,0\nH Field I encoding:1,1\nH Field P predictor:1,1\nH Field P encoding:0,0\nIgarbage\xff\xfe")
        lg = bfl.Log(p); lg.decode(); self.assertFalse(bfl.analyze(lg)["ok"])
        v = AnalysisView(); v.show_log(p); _spin(3, lambda: "Decoding" not in v.title.text()); self.assertNotIn("Decoding", v.title.text())

    def test_analysis_view_on_real_log(self):
        from nimbus_gcs import fc_logs as FL
        import glob
        hit = glob.glob(os.path.join(FL.ROOT, "archive", "flight_logs_2026-11-04_to_2026-11-12", "fc_blackbox", "20261106", "LOG00019_*.BFL"))   # the 2026-11-06 burst log
        if not hit: self.skipTest("LOG00019 of 2026-11-06 not saved")
        from nimbus_gcs.analysis_page import AnalysisView
        from PyQt6.QtWidgets import QCheckBox, QMessageBox
        v = AnalysisView(); v.show_log(hit[0]); _spin(30, lambda: bool(v.charts[0].t)); v.resize(900, 700); v.grab()
        self.assertTrue(v.charts[0].t); self.assertEqual(len(v.charts), 4); self.assertIn("burst", v.findings.text().lower())
        self.assertEqual(v.findChildren(QCheckBox), [])


def _spin(secs, cond=None):
    end = time.time() + secs
    while time.time() < end and not (cond and cond()):
        _app.processEvents(); time.sleep(0.02)


def _no_dialogs():
    from PyQt6.QtWidgets import QMessageBox
    saved = {n: QMessageBox.__dict__[n] for n in ("question", "information", "warning", "critical")}

    def boom(*a, **k):
        raise AssertionError("a QMessageBox was created")
    for n in saved:
        setattr(QMessageBox, n, staticmethod(boom))
    return QMessageBox, saved


class TestFlyPage(unittest.TestCase):
    def setUp(self):
        self.qm, self.saved = _no_dialogs()
        self.w = MainWindow()

    def tearDown(self):
        for n, v in self.saved.items():
            setattr(self.qm, n, v)
        self.w.model.disconnect(); self.w.close()

    def test_offline_buttons_disabled_and_pressing_never_opens_a_dialog(self):
        w = self.w; w.refresh(); f = w.fly
        for b in (f.btn_main, f.btn_disarm, f.btn_land):
            self.assertFalse(b.is_ready()); self.assertTrue(b.reason())
        self.assertTrue(w.btn_safe.isEnabled())
        f.btn_main.trigger(); f.btn_disarm.click(); f.btn_land.click(); w.btn_safe.click()
        self.assertIn("Not connected", f.why.text())
        self.assertTrue(f.status.text())

    def test_no_checkbox_advanced_or_old_controls(self):
        from PyQt6.QtWidgets import QCheckBox
        w = self.w
        self.assertEqual(w.findChildren(QCheckBox), [])
        for n in ("chk_arm", "box_xy", "source_box"):
            self.assertFalse(hasattr(w, n), n)

    def test_esc_is_safe_and_safe_is_always_enabled(self):
        w = self.w; w.show()
        calls = []
        orig = w.model.core.director.safe
        w.model.core.director.safe = lambda: (calls.append(1), orig())[1]
        self.assertEqual(w.sc_safe.key().toString(), "Esc"); w.sc_safe.activated.emit()
        self.assertEqual(calls, [1])
        for i in range(5):
            w.tabs.select(i); self.assertTrue(w.btn_safe.isEnabled()); self.assertTrue(w.btn_safe.isVisible())

    def test_sim_arm_takeoff_hold_land_through_the_director(self):
        w = self.w; f = w.fly
        import dataclasses                                       # measured flight profile set BEFORE the simulator is built (the tests' temporary settings are the unmeasured defaults, hover 0.42)
        _au = w.model.core.director._au; w.model.core.director.apply_settings(dataclasses.replace(_au.settings, thr_hover0=0.63, batt_ref_v=13.8, measured=True), persist=False)
        w.model.connect_sim(); _spin(5, lambda: f.director.state().phase == "ready"); _spin(0.5)         # the GUI button was removed; the model-level simulator still backs this test
        self.assertEqual(f.preflight._rows.__len__(), len(f.director.preflight()))
        self.assertTrue(f.btn_main.is_ready()); self.assertEqual(f.btn_main.label(), "ARM"); self.assertFalse(f.btn_land.is_ready()); self.assertFalse(f.btn_disarm.is_ready())
        f.height_field.set_value(0.5)
        f.btn_main.trigger()                                                           # ARM (hold time is 0 in tests)
        _spin(20, lambda: f.director.state().phase == "armed")
        self.assertEqual(f.director.state().phase, "armed"); _spin(3.0)               # the simulator starts its range stream at arm: the controller needs ~2 s of data
        self.assertTrue(f.btn_disarm.is_ready()); self.assertEqual(f.btn_main.label(), "TAKE OFF")
        f.btn_main.trigger()
        _spin(60, lambda: f.director.state().phase == "holding"); _spin(1)
        st = f.director.state()
        self.assertEqual(st.phase, "holding"); self.assertAlmostEqual(st.height_m, 0.5, delta=0.15)
        self.assertTrue(f.btn_land.is_ready()); self.assertFalse(f.btn_main.is_ready())
        self.assertIn("HOLD", f.phase_lbl.text()); self.assertGreater(len(f.plot.points()), 5)
        f.height_field.set_value(0.7, emit=True)                                       # in flight: changes the held height
        self.assertAlmostEqual(f.director.state().target_m, 0.7, places=2)
        f.btn_land.click()
        self.assertEqual(w.toasts.count(), 0)     # no Undo toast: an undo would re-arm or take off with one click
        _spin(40, lambda: f.director.state().phase in ("landed", "ready"))
        self.assertIn(f.director.state().phase, ("landed", "ready")); self.assertFalse(w.model.core.sess.sim.crashed)

    def test_top_bar_chips_bell_palette_and_toasts(self):
        w = self.w; w.show(); w.refresh()
        self.assertEqual(w.tabs._labels, ["FLY", "VIDEO", "SETUP", "ANALYZE", "SETTINGS"])
        self.assertEqual(w.chip_arm.value(), "--"); self.assertEqual(w.src_label.text(), "OFFLINE")
        w.model.core.notify("Test caution", "caution", "test"); w.refresh()
        self.assertGreaterEqual(w.bell.count(), 1)
        w.show_notifications(); self.assertTrue(any("Test caution" in w.notes.list.item(i).text() for i in range(w.notes.list.count())))
        self.assertEqual(w.bell.count(), 0); w.notes.hide()
        w.open_palette(); labels = w.palette.labels()
        self.assertIn("Go to SETUP / Radio", labels); self.assertFalse(any("arm" in x.lower().split("action:")[-1].split() for x in labels))
        w.palette.edit.setText("setup radio"); self.assertEqual(w.palette.labels(), ["Go to SETUP / Radio"])
        self.assertTrue(w.palette.run_current()); self.assertEqual((w.current_workspace(), w.setup_page.nav.current()), ("SETUP", "radio"))
        w.palette.hide()
        w.sc_palette.activated.emit(); self.assertTrue(w.palette.isVisible()); w.palette.hide()
        s = {"mode": "STAB*", "connected": True, "sim": False, "bench": False, "lq": 100, "rssi": -50, "battery_pct": 20.0, "battery_v": 13.8}
        w._refresh_chips(s); self.assertEqual(w.chip_batt.level(), "caution"); self.assertEqual(w.chip_arm.value(), "DISARMED")   # battery: amber warning only
        self.assertNotEqual(w.chip_batt.level(), "alarm")
        w.toasts.show_toast("x", lambda: None); self.assertGreaterEqual(w.toasts.count(), 1)

    def test_preflight_helpers(self):
        from nimbus_gcs.fly_page import check_mark, disabled_reason, first_blocker
        from nimbus_core.director import Check
        self.assertEqual(check_mark(True, True), ("✔", "ok")); self.assertEqual(check_mark(False, True), ("✖", "blocking")); self.assertEqual(check_mark(False, False), ("⚠", "caution"))
        cs = [Check("a", "Link", False, "LQ 10", False), Check("b", "Range", False, "none", True)]
        self.assertIn("Range", first_blocker(cs)); self.assertIn("Range", disabled_reason("ready", cs)); self.assertEqual(disabled_reason("ready", []), "")


