import os, unittest
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
from PyQt6.QtWidgets import QApplication
from nimbus_gcs.viz.drone_3d import Drone3DView

_app = QApplication.instance() or QApplication([])


class TestDrone3D(unittest.TestCase):
    def test_states_orbit_and_click(self):
        v = Drone3DView(); v.resize(600, 500); v.show(); got = []
        v.motor_clicked.connect(got.append)
        v.grab(); self.assertTrue(v.stale)
        v.set_outputs([1100] * 4, False); v.set_selected(2); v.set_seen([None, False, None, None]); v.grab()
        self.assertTrue(v.running()); y = v.yaw; v.yaw += 1; v.reset_view(); self.assertEqual(v.yaw, y)
        c, _r = v._hit[3]; v.click_at(c); self.assertEqual(got, [3])
        v.set_outputs(None, True); self.assertTrue(v.stale)


class FakeMT:
    def __init__(self):
        self.state = "idle"; self.motor = 0; self.readback = None; self.t_readback = 0.0; self.status_text = ""; self.unconfirmed = False; self.gate = ""; self.log = []
        self.value_us = 1075; self.lq = 100; self.armed = False

    def refusal(self): return self.gate

    def start(self, n):
        if self.gate: return self.gate
        self.state, self.motor = "spinning", n; self.log.append(("start", n)); return ""

    def refresh(self): self.log.append(("refresh",))

    def stop(self, reason="release", blocking=False):
        if self.state != "idle": self.log.append(("stop", reason)); self.state = "idle"


class TestMotorPage(unittest.TestCase):
    """SETUP > Motor test, 2026-11-24 design after DRONE_GCS (frame geometry | FC outputs | bench motor test)."""
    def _tab(self):
        import time
        from nimbus_gcs.main_window import MainWindow
        w = MainWindow(); w.resize(1366, 768); w.show(); w.build_all(); w.goto("SETUP", "Motor test")
        return w, w.motor_tab, time

    def test_layout_selection_outputs_and_rules(self):
        from PyQt6.QtWidgets import QCheckBox, QComboBox, QMessageBox, QPushButton, QLabel
        w, t, time = self._tab()
        try:
            self.assertEqual([b.n for b in t.mbtn], [1, 2, 3, 4]); self.assertIn("rear right", t.mbtn[0].label)
            t.mbtn[2].click(); self.assertEqual(t.view.selected, 3); self.assertTrue(t.mbtn[2].isChecked() and not t.mbtn[0].isChecked())
            t.view.grab(); c, _r = t.view._hit[2]; t.view.click_at(c); self.assertEqual(t.view.selected, 2); self.assertTrue(t.mbtn[1].isChecked() and not t.mbtn[2].isChecked())
            t.outputs = [1000, 1050, 1100, 1020]; t.t_out = time.monotonic(); t.refresh()
            self.assertEqual([b.us for b in t.mbtn], [1000, 1050, 1100, 1020]); self.assertFalse(t.mbtn[0].stale); self.assertAlmostEqual(t.mbtn[2].percent(), 10.0)
            self.assertEqual([t.bars[n].us for n in (1, 2, 3, 4)], [1000, 1050, 1100, 1020]); self.assertEqual(t.out_meta.text(), "LIVE")
            t.outputs = None; t.refresh(); self.assertTrue(all(b.stale for b in t.mbtn)); self.assertTrue(all(b.stale for b in t.bars.values())); self.assertEqual(t.out_meta.text(), "NO DATA")
            texts = [x.text() for x in t.findChildren(QPushButton) if x.text()]
            for want in ("HOLD TO SPIN", "SEQ 1", "STOP ALL", "ARM + SPIN", "STOP / DISARM", "HOLD 1 s TO UNLOCK"):
                self.assertTrue(any(x.startswith(want) for x in texts), want)
            self.assertTrue(any(l.text().startswith("THIS ARMS THE DRONE") for l in t.findChildren(QLabel)))
            self.assertTrue(any(l.text().startswith("THIS SPINS THE MOTOR YOU HOLD:") for l in t.findChildren(QLabel)))
            self.assertFalse(t.findChildren(QCheckBox)); self.assertFalse(t.findChildren(QComboBox)); self.assertFalse(QMessageBox in type(t).__mro__)
            for a in ("btn_arm", "btn_stop", "sp", "on_arm", "on_stop", "running", "refresh", "view", "stop_spin", "select_motor"):
                self.assertTrue(hasattr(t, a))
            t.sp.setValue(3.0); self.assertIn("1030", t.thr_val.text()); t.sp.setValue(99); self.assertEqual(t.sp.value(), 10.0)      # the slider never passes the ceiling
            from nimbus_gcs import theme
            src = open(__import__("nimbus_gcs.motor_tab", fromlist=["x"]).__file__).read()
            import re
            self.assertEqual(re.findall(r"#[0-9A-Fa-f]{6}", src), [])                                                   # colours only from theme.py
        finally:
            w.model.disconnect(); w.close()

    def test_locked_until_unlock_then_hold_spins_and_release_stops(self):
        w, t, time = self._tab()
        try:
            f = FakeMT(); w.model.core.motor_test = f
            t.select_motor(2); t.on_hold_press(); self.assertEqual(f.log, []); self.assertIn("Locked", t.note.text())
            t.refresh(); self.assertFalse(t.btn_hold.isEnabled()); self.assertFalse(t.btn_seq.isEnabled())
            t.unlock(); t.refresh(); self.assertTrue(t.btn_hold.isEnabled()); self.assertTrue(t.g_unlock.ok)
            t.on_hold_press(); self.assertEqual(f.log[0], ("start", 2))
            f.readback = [1000, 1075, 1000, 1000]; f.t_readback = time.monotonic(); t.refresh()
            self.assertTrue(t.mbtn[1].spinning and not t.mbtn[0].spinning); self.assertEqual(t.view.outputs, [1000, 1075, 1000, 1000]); self.assertEqual(t.view.spinning, 2)
            self.assertIn("SPINNING", t.status.text()); self.assertIn("M2 SPINNING", t.chip.text())
            t.on_release(); self.assertEqual(f.log[-1], ("stop", "release")); self.assertFalse(any(b.spinning for b in t.mbtn))
            f.gate = "FC is armed: disarm first."; t.refresh()
            self.assertFalse(t.btn_hold.isEnabled()); self.assertIn("FC is armed: disarm first.", t.note.text()); self.assertEqual(t.chip.text(), "LOCKED")
            t.on_hold_press(); self.assertEqual(f.log[-1], ("stop", "release"))                                       # the core refuses: nothing started
            f.gate = ""; t.on_hold_press(); t.stop_spin("tab change"); self.assertEqual(f.log[-1], ("stop", "tab change"))
            t.on_hold_press(); w.goto("FLY"); self.assertEqual(f.log[-1], ("stop", "tab change"))
        finally:
            w.model.disconnect(); w.close()

    def test_sequence_runs_m1_to_m4_and_stop_all_locks(self):
        import nimbus_gcs.motor_tab as M
        from types import SimpleNamespace
        w, t, time = self._tab(); clock = [1000.0]; real = M.time
        try:
            M.time = SimpleNamespace(monotonic=lambda: clock[0])
            f = FakeMT(); w.model.core.motor_test = f
            t.toggle_sequence(); self.assertFalse(t.seq_on)                                                              # locked
            t.unlock(); t.toggle_sequence(); self.assertTrue(t.seq_on); self.assertEqual(f.log[0], ("start", 1))
            for _ in range(150):
                clock[0] += 0.1; t._tick_hold()
                if not t.seq_on: break
            starts = [e[1] for e in f.log if e[0] == "start"]; self.assertEqual(starts, [1, 2, 3, 4]); self.assertFalse(t.seq_on)
            self.assertIn("Sequence done", t.note.text())
            t.toggle_sequence(); self.assertTrue(t.seq_on); t.stop_all()
            self.assertFalse(t.seq_on); self.assertFalse(t.unlocked()); self.assertEqual(f.state, "idle"); self.assertIn("Unlock again", t.note.text())
            clock[0] += 61; t.unlock(); clock[0] += 61; self.assertFalse(t.unlocked())                                   # the unlock lapses by itself
        finally:
            M.time = real; w.model.disconnect(); w.close()


if __name__ == "__main__":
    unittest.main()


class TestArmSpinAndRateRace(unittest.TestCase):
    def test_arm_and_spin_button_runs_without_the_removed_widgets(self):
        """Real incident 2026-11-13: on_arm used win.source_box / win.chk_arm, which no longer exist -> AttributeError after the source was already changed."""
        from nimbus_gcs.main_window import MainWindow
        w = MainWindow(); w.resize(1366, 768); w.show(); w.build_all(); w.goto("SETUP", "Motor test")
        try:
            w.model.core.connect_sim()
            t = w.motor_tab
            self.assertEqual(t._block_reason(), "")
            t.on_arm()                                                  # must not raise
            self.assertEqual(w.model.source, "external"); self.assertTrue(w.model.allow_arm)
            self.assertIn("Motor test started", t.note.text())
            t.on_stop()
        finally:
            w.model.disconnect(); w.close()

    def test_frame_rate_counter_survives_a_writer_thread(self):
        """Real incident 2026-11-13: RuntimeError 'deque mutated during iteration' on the Radio frames page."""
        import collections, threading
        from nimbus_pc import live
        st = live.LiveState() if hasattr(live, "LiveState") else None
        self.assertIsNotNone(st)
        q = collections.deque(maxlen=4000); table = {0xD6: q}; stop = threading.Event()

        def writer():
            while not stop.is_set():
                q.append(st.clock())
        th = threading.Thread(target=writer, daemon=True); th.start()
        try:
            for _ in range(3000):
                st.rate(table, 0xD6)                                    # raised RuntimeError before the fix
        finally:
            stop.set(); th.join(2)
