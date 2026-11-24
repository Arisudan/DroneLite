import unittest

from nimbus_pc.hover.controller import HoverController, HoverState


def ctl(z=0.5, q=200, lifted=True, vx=0.0, vy=0.0, x=0.0, y=0.0, now=10.0):
    c = HoverController()
    c.state = HoverState.HOVER; c.sign_done = True
    c.lifted = lifted; c.kz.x[0] = z
    c.kx.x[:] = [x, vx]; c.ky.x[:] = [y, vy]
    c.flow_raw = [(vx, vy), (vx, vy)]
    c._flow_q, c._flow_t, c._now = q, now, now
    return c


def step(c, dt=0.1, **kw):
    """Advance time, optionally refreshing flow data and overriding estimate fields."""
    c._now += dt
    if kw.get("fresh", True): c._flow_t = c._now
    if "q" in kw: c._flow_q = kw["q"]
    if "z" in kw: c.kz.x[0] = kw["z"]
    if "v" in kw:
        c.kx.x[1], c.ky.x[1] = kw["v"]; c.flow_raw = [kw["v"], kw["v"]]
    if "pos" in kw: c.kx.x[0], c.ky.x[0] = kw["pos"]
    return c._horizontal(dt)


class TestXyGate(unittest.TestCase):
    def test_closed_on_floor(self):
        c = ctl(lifted=False, x=2.0)
        self.assertEqual(step(c), (0.0, 0.0)); self.assertFalse(c.xy_gate_open); self.assertIn("floor", c.xy_gate_reason)

    def test_closed_at_low_height(self):
        c = ctl(z=0.12, x=2.0)
        self.assertEqual(step(c), (0.0, 0.0)); self.assertFalse(c.xy_gate_open)
        c = ctl(z=0.19)
        step(c); self.assertFalse(c.xy_gate_open)

    def test_closed_on_low_flow_quality_and_stale_flow(self):
        c = ctl(q=40, x=1.0)
        self.assertEqual(step(c), (0.0, 0.0)); self.assertIn("quality", c.xy_gate_reason)
        c = ctl()
        step(c); self.assertTrue(c.xy_gate_open)
        step(c, dt=0.7, fresh=False); self.assertFalse(c.xy_gate_open); self.assertIn("stale", c.xy_gate_reason)

    def test_closed_on_high_speed_for_at_least_half_a_second(self):
        c = ctl()
        step(c); self.assertTrue(c.xy_gate_open)
        self.assertEqual(step(c, v=(1.5, 0.0), pos=(0.3, 0.0)), (0.0, 0.0)); self.assertFalse(c.xy_gate_open)
        for _ in range(3):                                      # speed back to normal, still inside the 0.5 s hold-off
            self.assertEqual(step(c, v=(0.0, 0.0)), (0.0, 0.0)); self.assertFalse(c.xy_gate_open)
        for _ in range(4): step(c, v=(0.0, 0.0))
        self.assertTrue(c.xy_gate_open)

    def test_height_hysteresis(self):
        c = ctl(z=0.25)
        step(c); self.assertTrue(c.xy_gate_open)
        step(c, z=0.17); self.assertTrue(c.xy_gate_open)        # between 0.15 and 0.20: stays open
        step(c, z=0.14); self.assertFalse(c.xy_gate_open)
        step(c, z=0.17); self.assertFalse(c.xy_gate_open)       # must climb back above 0.20
        step(c, z=0.21); self.assertTrue(c.xy_gate_open)

    def test_no_jump_on_opening_and_hold_acts_afterwards(self):
        c = ctl(z=0.1, x=3.0, y=-2.0)
        self.assertEqual(step(c), (0.0, 0.0))
        step(c, z=0.3)                                          # opens here, 3 m from the old reference
        self.assertTrue(c.xy_gate_open)
        self.assertEqual((c.x_ref, c.y_ref), (3.0, -2.0))
        self.assertEqual(step(c), (0.0, 0.0))                   # at the reference: no command
        p, r = step(c, pos=(3.4, -2.0))                         # drifted 0.4 m: hold now commands a correction
        self.assertNotEqual(p, 0.0)
        self.assertTrue(any("gate opened" in e[1] for e in c.events))

    def test_phantom_velocity_from_the_recording_gives_zero_command(self):
        c = ctl(z=0.07, q=90, vx=1.5, vy=1.0, x=4.0, y=2.0)
        for _ in range(20):
            self.assertEqual(step(c, v=(1.5, 1.0), pos=(4.0, 2.0)), (0.0, 0.0))
        self.assertFalse(c.xy_gate_open)
        self.assertEqual(c.ix, [0.0, 0.0])
        # even at a good height, a 1.5 m/s phantom keeps it closed
        self.assertEqual(step(c, z=0.5), (0.0, 0.0)); self.assertFalse(c.xy_gate_open)


if __name__ == "__main__":
    unittest.main()
