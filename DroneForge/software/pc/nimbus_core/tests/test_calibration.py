import math
import unittest
from types import SimpleNamespace

from nimbus_core.calibration import (Calibrator, tilt_colour, gyro_verdict, LEVEL_GESTURE_S, NOT_LEVEL)


class FakeCore:
    def __init__(self, **kw):
        self.connected, self.state_source, self.source = True, "live", "idle"
        self.autonomy = SimpleNamespace(fc_armed=False, phase="idle")
        self.ramp = SimpleNamespace(phase="idle")
        self.sent, self.sources = [], []
        self.__dict__.update(kw)

    def set_source(self, s): self.source = s; self.sources.append(s)
    def set_external_controls(self, c): self.sent.append(c)


class Clock:
    def __init__(self): self.t = 100.0
    def __call__(self): return self.t
    def sleep(self, d): self.t += d


def att(cal, t, p, r):
    cal.observe("attitude", {"pitch_rad": math.radians(p), "roll_rad": math.radians(r)}, now=t)


def ready(core=None, p=0.4, r=-0.2, lq=90, batt=15.0, mode="STAB*", still=True):
    """A Calibrator whose telemetry (all fresh, drone still) lets level calibration start."""
    clk, core = Clock(), core or FakeCore()
    cal = Calibrator(core, clock=clk, sleep=clk.sleep)
    for i in range(6):
        att(cal, clk.t - 2 + i * 0.4, p + (0 if still or i % 2 else 1.0), r)
    cal.observe("flight_mode", {"mode": mode}, now=clk.t)
    cal.observe("link", {"up_lq": lq}, now=clk.t)
    if batt is not None:
        cal.observe("battery", {"voltage_v": batt}, now=clk.t)
    return cal, core, clk


class TestLevelRefusals(unittest.TestCase):
    def test_ready_ok(self):
        self.assertIsNone(ready()[0].level_refusal())

    def test_every_refusal(self):
        cases = []
        c, *_ = ready(core=FakeCore(connected=False)); cases.append((c, "connect"))
        c, *_ = ready(core=FakeCore(state_source="sim")); cases.append((c, "simulator"))
        c, *_ = ready(mode="STAB"); cases.append((c, "armed"))
        c, *_ = ready(core=FakeCore(autonomy=SimpleNamespace(fc_armed=False, phase="flying"))); cases.append((c, "flight"))
        c, *_ = ready(core=FakeCore(ramp=SimpleNamespace(phase="ramping"))); cases.append((c, "ramp"))
        c, *_ = ready(core=FakeCore(source="gamepad")); cases.append((c, "source"))
        c, *_ = ready(lq=40); cases.append((c, "Link quality"))
        c, *_ = ready(batt=None); cases.append((c, "battery"))
        c, *_ = ready(still=False); cases.append((c, "not still"))
        c, _, clk = ready(); clk.t += 10; cases.append((c, "No fresh"))             # everything stale
        c, *_ = ready(); c._t_status = None; cases.append((c, "status"))
        for cal, word in cases:
            why = cal.level_refusal()
            self.assertIsNotNone(why, word); self.assertIn(word.lower(), why.lower())
            ok, msg = cal.calibrate_level()
            self.assertFalse(ok); self.assertEqual(cal.running, False); self.assertEqual(cal.core.sent, [])

    def test_stale_attitude_and_status_separately(self):
        c, _, clk = ready()
        c.observe("flight_mode", {"mode": "STAB*"}, now=clk.t + 9); c.observe("link", {"up_lq": 90}, now=clk.t + 9); c.observe("battery", {"voltage_v": 15}, now=clk.t + 9)
        clk.t += 9
        self.assertIn("attitude", c.level_refusal())

    def test_gesture_never_with_busy_core(self):
        c, core, _ = ready(); c.running = True
        self.assertIn("already", c.level_refusal())


class TestLevelRun(unittest.TestCase):
    def _run(self, after=(0.1, -0.1), core=None, hook=None):
        cal, core, clk = ready(core=core)
        before = (0.4, -0.2)
        orig = core.set_external_controls

        def send(c):
            orig(c)
            if hook: hook(core, c)
            att(cal, clk.t, *after)          # attitude keeps arriving during the run (read back afterwards)
        core.set_external_controls = send
        cal.running, cal._prev_source = True, "idle"
        cal._run_level(before)
        return cal, core

    def test_success_before_after(self):
        cal, core = self._run()
        r = cal.level_result
        self.assertTrue(r["ok"]); self.assertEqual(r["before"], (0.4, -0.2)); self.assertAlmostEqual(r["after"][0], 0.1, 2)
        self.assertIn("pitch +0.4 deg", r["text"]); self.assertIn("pitch +0.1 deg", r["text"])
        g = [x for x in core.sent if x.throttle == 1.0]
        self.assertTrue(g); self.assertAlmostEqual(len(g) * 0.02, LEVEL_GESTURE_S, delta=0.1)
        for x in g:
            self.assertEqual((x.pitch, x.yaw, x.roll, x.arm), (-1.0, -1.0, 0.0, False))
        self.assertTrue(all(not x.arm for x in core.sent))                           # never arms
        last = core.sent[-1]; self.assertEqual((last.throttle, last.pitch, last.yaw, last.arm), (0.0, 0.0, 0.0, False))
        self.assertEqual(core.source, "idle"); self.assertFalse(cal.running)

    def test_not_level(self):
        cal, core = self._run(after=(0.8, 0.1))
        self.assertFalse(cal.level_result["ok"]); self.assertIn(NOT_LEVEL, cal.level_result["text"])
        self.assertEqual(core.sent[-1].throttle, 0.0)

    def test_safe_midway_restores(self):
        def hook(core, c):
            if len(core.sent) == 30: core.source = "idle"                           # SAFE pressed
        cal, core = self._run(hook=hook)
        self.assertFalse(cal.level_result["ok"]); self.assertIn("stopped", cal.level_result["text"])
        self.assertEqual(core.sent[-1].throttle, 0.0); self.assertFalse(cal.running)

    def test_disconnect_and_armed_stop(self):
        def hook(core, c):
            if len(core.sent) == 10: core.connected = False
        cal, core = self._run(hook=hook)
        self.assertIn("link dropped", cal.level_result["text"]); self.assertEqual(core.sent[-1].throttle, 0.0)
        cal, core = self._run(hook=lambda core, c: setattr(core.autonomy, "fc_armed", True) if len(core.sent) == 5 else None)
        self.assertIn("armed", cal.level_result["text"]); self.assertEqual(core.sent[-1].throttle, 0.0)

    def test_error_restores_controls(self):
        core = FakeCore()
        def boom(core, c):
            if len(core.sent) == 20: raise RuntimeError("x")
        cal, core = self._run(core=core, hook=boom)
        self.assertIn("failed", cal.level_result["text"]); self.assertEqual(core.sent[-1].throttle, 0.0); self.assertFalse(cal.running)

    def test_start_runs_thread(self):
        cal, core, clk = ready()
        orig = core.set_external_controls
        core.set_external_controls = lambda c: (orig(c), att(cal, clk.t, 0.0, 0.0))
        ok, _ = cal.calibrate_level(); self.assertTrue(ok); cal._thread.join(10)
        self.assertFalse(cal.running); self.assertTrue(cal.level_result["ok"])


class TestGyro(unittest.TestCase):
    def _feed(self, cal, clk, vals, n=20, dt=0.2):
        for i in range(n):
            clk.t += dt
            cal.observe("flight_mode", {"mode": "STAB*", "gyro_dps10": [int(v * 10) for v in vals(i)]}, now=clk.t)

    def _check(self, vals, n=20):
        cal, core, clk = ready()
        ok, _ = cal.start_gyro_check(); self.assertTrue(ok)
        self._feed(cal, clk, vals, n); clk.t += (0.1 if n > 10 else 4.0)
        return cal.gyro_poll()

    def test_verdicts(self):
        self.assertEqual(self._check(lambda i: (0.1, -0.1, 0.0))["verdict"], "OK")
        self.assertEqual(self._check(lambda i: (0.7, 0.0, 0.0))["verdict"], "CAUTION")
        self.assertEqual(self._check(lambda i: (1.5, 0.0, 0.0))["verdict"], "NEEDS RE-CAL")
        self.assertEqual(self._check(lambda i: (0.0, 0.0, 1.0 if i % 2 else -1.0))["verdict"], "NEEDS RE-CAL")   # noise
        self.assertEqual((gyro_verdict(0.2, 0.1), gyro_verdict(0.6, 0.1), gyro_verdict(0.1, 0.6)), ("OK", "CAUTION", "NEEDS RE-CAL"))

    def test_no_data_said_plainly(self):
        r = self._check(lambda i: (0, 0, 0), n=3)
        self.assertEqual(r["verdict"], ""); self.assertIn("No gyro data", r["text"])

    def test_not_ready_until_3s_and_refusals(self):
        cal, core, clk = ready(); cal.start_gyro_check(); self._feed(cal, clk, lambda i: (0, 0, 0), n=5)
        self.assertIsNone(cal.gyro_poll()); self.assertTrue(cal.gyro_measuring())
        c2, *_ = ready(mode="STAB"); self.assertIn("armed", c2.start_gyro_check()[1])
        c3, *_ = ready(core=FakeCore(connected=False)); self.assertFalse(c3.start_gyro_check()[0])

    def test_armed_cancels(self):
        cal, core, clk = ready(); cal.start_gyro_check()
        cal.observe("flight_mode", {"mode": "STAB"}, now=clk.t)
        self.assertIn("cancelled", cal.gyro_poll()["text"])


class TestMisc(unittest.TestCase):
    def test_level_status_and_colours(self):
        cal, *_ = ready(p=0.3, r=-1.0)
        s = cal.level_status()
        self.assertAlmostEqual(s["pitch"], 0.3); self.assertEqual(s["colour"], "amber"); self.assertFalse(s["stale"])
        self.assertEqual((tilt_colour(0.2), tilt_colour(-1.6), tilt_colour(None)), ("green", "red", "grey"))
        cal2, _, clk = ready(); clk.t += 30
        s = cal2.level_status(); self.assertTrue(s["stale"]); self.assertEqual(s["colour"], "grey")
        self.assertIsNone(Calibrator(FakeCore()).level_status()["pitch"])


if __name__ == "__main__":
    unittest.main()
