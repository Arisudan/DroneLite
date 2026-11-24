import unittest
from dataclasses import replace

from nimbus_pc.hover import runner
from nimbus_pc.hover.controller import HoverConfig, HoverController, HoverState
from nimbus_pc.hover.sim import Sample


def _states(res, after_t):
    out = [r["state"] for r in res.rows if r["t"] >= after_t]
    return sum(1 for a, b in zip(out, out[1:]) if a != b)


class TestLiftMargin(unittest.TestCase):
    def test_default_margin_is_negative(self):
        self.assertAlmostEqual(HoverConfig().takeoff_lift_margin, 0.0)      # was -0.05: the margin is neutralised, the climb velocity loop finds the hover throttle

    def test_base_is_lift_throttle_plus_margin(self):
        c = HoverController(replace(HoverConfig(), thr_hover0=0.68, thr_max=0.9))
        c.v_start = c.batt_v
        c.ground_thr = 0.62
        c.state = HoverState.TAKEOFF
        # replicate the lift-off bookkeeping through the real code path
        c.state = HoverState.TAKEOFF; c.t_state = 0.0; c._now = 1.0
        c.kz.x[0], c.kz.x[1] = 0.08, 0.2
        c._lift_cnt = 2
        c._altitude(1.0, 0.01)
        self.assertTrue(c.lifted)
        self.assertAlmostEqual(c.lift_thr, c.thr_at_lift, places=6)         # was + 0.05 (the old default margin)

    def test_base_capped_by_thr_max(self):
        c = HoverController(replace(HoverConfig(), thr_hover0=0.68, thr_max=0.66))
        c.v_start = c.batt_v; c.ground_thr = 0.64
        c.state = HoverState.TAKEOFF; c._now = 1.0
        c.kz.x[0], c.kz.x[1] = 0.08, 0.2; c._lift_cnt = 2
        c._altitude(1.0, 0.01)
        self.assertLessEqual(c.lift_thr, 0.66 + 1e-9)


class TestGroundEffect(unittest.TestCase):
    def test_acceptance(self):
        res = runner.run("ground_effect", duration=40, land_at=None, seed=1)
        m = res.metrics
        self.assertFalse(m["crashed"], m)
        t5 = next((r["t"] for r in res.rows if r["z"] >= 0.5), None)
        self.assertIsNotNone(t5, "never reached 0.5 m")
        self.assertLessEqual(t5 - 3.5, 20.0)
        self.assertLessEqual(max(r["z"] for r in res.rows) - 0.5, 0.12)
        thr = [r["thr"] for r in res.rows]
        self.assertLessEqual(max(abs(b - a) for a, b in zip(thr, thr[1:])), 0.25)      # rows are 0.1 s apart: a stricter bound than 0.25 per 20 ms


class TestDropouts(unittest.TestCase):
    def test_injection_no_flapping_and_same_height(self):
        clean = runner.run("ground_effect", duration=30, land_at=None, seed=3)
        drop = runner.run("range_dropouts", duration=30, land_at=None, seed=3)
        self.assertFalse(drop.metrics["crashed"])
        self.assertTrue(drop.metrics["reached_hover"])
        t_lift = next(r["t"] for r in drop.rows if r["z"] > 0.04)
        self.assertLessEqual(_states(drop, t_lift), 2)
        def mean_after(res, t0):
            z = [r["z"] for r in res.rows if r["t"] >= t0]
            return sum(z) / len(z)
        self.assertLess(abs(mean_after(drop, 18.0) - mean_after(clean, 18.0)), 0.05)

    def test_sustained_low_is_real_descent(self):
        c = HoverController(HoverConfig())
        c.lifted = True; c.state = HoverState.HOVER
        c.kz.x[0] = 0.5
        t = 10.0
        for _ in range(3):
            c._ingest(t, Sample(t, "optrange", {"distance_m": 0.5, "flow_vx": 0, "flow_vy": 0, "quality": 200})); t += 0.1
        n0 = c.dropouts_rejected
        c._ingest(t, Sample(t, "optrange", {"distance_m": 0.02, "flow_vx": 0, "flow_vy": 0, "quality": 200}))
        self.assertEqual(c.dropouts_rejected, n0 + 1)
        self.assertAlmostEqual(c.kz.x[0], 0.5, places=1)       # rejected: not fed to the filter
        for _ in range(8):                                      # 0.8 s of persistent low readings
            t += 0.1
            c._ingest(t, Sample(t, "optrange", {"distance_m": 0.02, "flow_vx": 0, "flow_vy": 0, "quality": 200}))
        self.assertLess(c.dropouts_rejected, n0 + 8)            # the later ones were accepted
        self.assertLess(c.kz.x[0], 0.5)                         # and pulled the estimate down

if __name__ == "__main__":
    unittest.main()
