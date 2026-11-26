import csv, os, tempfile, unittest
from nimbus_core.core import NimbusCore
from nimbus_core.hold_log import COLS, HoldLog
from nimbus_pc.sender import Controls


class TestHoldLog(unittest.TestCase):
    def test_rows_columns_and_close_on_disconnect(self):
        d = tempfile.mkdtemp(); core = NimbusCore(); core.connect_sim(); self.addCleanup(core.disconnect)
        t = [0.0]; hl = HoldLog(core, d, clock=lambda: t[0])
        core.vo_status = {"anchor": True, "fresh": False, "text": "position aid: no fresh VO"}
        core.pos_aid = (0.12, -0.05, 0.0)
        for i in range(30): t[0] += 0.1; hl.sample(Controls(arm=True))
        hl.close()
        rows = list(csv.reader(open(hl.path)))
        self.assertEqual(rows[0], COLS); self.assertGreaterEqual(len(rows), 20)
        r = dict(zip(COLS, rows[-1])); self.assertEqual(r["core_aid_fwd_m"], "0.12"); self.assertEqual(r["vo_anchor"], "1"); self.assertEqual(r["sent_arm"], "1")
        core.disconnect(); n = len(rows); hl.sample(Controls()); self.assertIsNone(hl.f)

    def test_disarmed_is_slow(self):
        d = tempfile.mkdtemp(); core = NimbusCore(); core.connect_sim(); self.addCleanup(core.disconnect)
        t = [0.0]; hl = HoldLog(core, d, clock=lambda: t[0])
        for i in range(30): t[0] += 0.1; hl.sample(Controls())
        hl.close(); self.assertLessEqual(len(list(csv.reader(open(hl.path)))), 14)


if __name__ == "__main__":
    unittest.main()


class TestProbeQuietWhenArmed(unittest.TestCase):
    def test_probe_asks_less_when_armed(self):
        import time
        from nimbus_core.radio_fc import RadioFcProbe, ARMED_PERIOD_S
        class S:
            n = []
            def request_msp(self, cmd, timeout=0.3): S.n.append(cmd); return None
        p = RadioFcProbe(S(), period_s=0.01); p.armed = True; p.start(); time.sleep(0.3); p.stop()
        self.assertLessEqual(len(S.n), 2); self.assertTrue(all(c == 105 for c in S.n)); self.assertGreater(ARMED_PERIOD_S, 1.0)
