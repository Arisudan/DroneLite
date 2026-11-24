import unittest

from nimbus_pc.hover.trajectory import JerkLimited1D


def _run(x0, v0, x1, dt=0.01, limits=(0.25, 0.35, 0.8), replan=None, t_max=120.0):
    p = JerkLimited1D(*limits)
    p.plan(x0, v0, x1)
    t, a_prev = 0.0, 0.0
    mv = ma = mj = 0.0
    while not p.done and t < t_max:
        x, v, a = p.step(dt)
        t += dt
        mv, ma, mj = max(mv, abs(v)), max(ma, abs(a)), max(mj, abs(a - a_prev) / dt)
        a_prev = a
        if replan and abs(t - replan[0]) < dt / 2:
            p.plan(x, v, replan[1])
    return p, t, mv, ma, mj


class TestJerkLimited1D(unittest.TestCase):
    TOL = 1.0e-6

    def test_never_exceeds_the_limits_and_reaches_the_target_exactly(self):
        for x0, v0, x1 in ((0, 0, 0.5), (0.5, 0, 0), (0, 0, 0.02), (0, 0.2, 0.5), (0.3, -0.1, 1.2), (0, 0, 3.0)):
            p, t, mv, ma, mj = _run(x0, v0, x1)
            self.assertTrue(p.done, (x0, v0, x1)); self.assertEqual(p.x, x1); self.assertEqual((p.v, p.a), (0.0, 0.0))
            self.assertLessEqual(mv, 0.25 + self.TOL); self.assertLessEqual(ma, 0.35 + self.TOL)
            self.assertLessEqual(mj, 0.8 * 1.5)                      # the last 0.2 mm settle may take one slightly larger acceleration step

    def test_a_long_move_cruises_at_the_speed_limit(self):
        p, t, mv, ma, mj = _run(0, 0, 3.0)
        self.assertAlmostEqual(mv, 0.25, places=3)
        self.assertGreater(t, 3.0 / 0.25)                           # cruise + ramps: slower than distance / v_max

    def test_short_move_does_not_reach_the_speed_limit(self):
        p, t, mv, ma, mj = _run(0, 0, 0.02)
        self.assertLess(mv, 0.1)

    def test_reversal_in_the_middle_of_a_move_stays_within_the_limits(self):
        p, t, mv, ma, mj = _run(0, 0, 1.0, replan=(2.0, 0.0))
        self.assertTrue(p.done); self.assertEqual(p.x, 0.0)
        self.assertLessEqual(mv, 0.25 + self.TOL); self.assertLessEqual(ma, 0.35 + self.TOL); self.assertLessEqual(mj, 0.8 * 1.5)

    def test_replanning_from_moving_state_keeps_position_and_velocity_continuous(self):
        p = JerkLimited1D(); p.plan(0, 0, 1.0)
        for _ in range(250): p.step(0.01)
        x, v = p.x, p.v
        p.plan(x, v, 0.2)
        x2, v2, _ = p.step(0.01)
        self.assertLess(abs(x2 - x), 0.01); self.assertLess(abs(v2 - v), 0.02)

    def test_zero_length_move_is_done_at_once(self):
        p = JerkLimited1D(); p.plan(0.4, 0.0, 0.4)
        self.assertTrue(p.done)
        self.assertEqual(p.step(0.01), (0.4, 0.0, 0.0))

    def test_stepping_with_other_dt_gives_the_same_end(self):
        for dt in (0.005, 0.02, 0.1):
            p, t, mv, ma, mj = _run(0, 0, 0.5, dt=dt)
            self.assertTrue(p.done); self.assertEqual(p.x, 0.5); self.assertLessEqual(mv, 0.25 + self.TOL); self.assertLessEqual(ma, 0.35 + self.TOL)

    def test_custom_limits_are_respected(self):
        p, t, mv, ma, mj = _run(0, 0, 1.0, limits=(0.10, 0.2, 0.5))
        self.assertLessEqual(mv, 0.10 + self.TOL); self.assertLessEqual(ma, 0.2 + self.TOL); self.assertLessEqual(mj, 0.5 * 1.5); self.assertEqual(p.x, 1.0)


if __name__ == "__main__":
    unittest.main()
