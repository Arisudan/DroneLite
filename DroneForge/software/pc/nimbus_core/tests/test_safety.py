import unittest

from nimbus_core.safety import SafetyConfig, SafetyMonitor, landing_throttle


def mon(**kw):
    m = SafetyMonitor(SafetyConfig(**kw))
    m.feed_link(0.0, 100.0)
    return m


class TestTilt(unittest.TestCase):
    def test_extreme_tilt_disarms_at_once_and_latches_until_reset(self):
        m = mon(); m.feed_attitude(1.0, 75, 0)
        ev = m.check(1.0, armed=True)
        self.assertEqual([e.code for e in ev], ["tilt_extreme"]); self.assertEqual(ev[0].level, "disarm")
        self.assertEqual(m.disarm_latched, "tilt_extreme")
        m.feed_attitude(1.1, 0, 0); m.check(1.1, True)
        self.assertEqual(m.disarm_latched, "tilt_extreme")           # level again, still latched
        m.reset(); self.assertIsNone(m.disarm_latched)

    def test_sustained_tilt_needs_the_full_time_and_a_short_spike_is_ignored(self):
        m = mon(); t = 1.0
        m.feed_attitude(t, 50, 0); self.assertEqual(m.check(t, True), [])
        m.feed_attitude(t + 0.3, 50, 0); self.assertEqual(m.check(t + 0.3, True), [])
        m.feed_attitude(t + 0.35, 10, 0); m.check(t + 0.35, True)    # back under the limit: timer restarts
        m.feed_attitude(t + 0.4, 50, 0); self.assertEqual(m.check(t + 0.4, True), [])
        m.feed_attitude(t + 0.95, 50, 0)
        ev = m.check(t + 0.95, True)
        self.assertEqual([e.code for e in ev], ["tilt_sustained"])

    def test_inverted_disarms(self):
        m = mon(); m.feed_attitude(1.0, 0, 150)
        self.assertEqual(m.check(1.0, True)[0].code, "inverted")

    def test_no_tilt_disarm_when_not_armed_in_turtle_mode_or_with_stale_attitude(self):
        m = mon(); m.feed_attitude(1.0, 80, 0)
        self.assertEqual(m.check(1.0, armed=False), [])
        self.assertEqual(m.check(1.0, armed=True, turtle=True), [])
        self.assertEqual(m.check(5.0, armed=True), [])                # attitude 4 s old


class TestBattery(unittest.TestCase):
    def test_default_low_battery_is_a_warning_only_no_landing_request(self):
        m = mon()
        for i in range(30): m.feed_battery(i * 0.1, 14.0); m.check(i * 0.1, True)
        t = 3.0; codes = []
        for i in range(1, 120):
            t += 0.1; m.feed_battery(t, 13.2); m.feed_link(t, 100); codes += [(e.code, e.level) for e in m.check(t, True)]
        self.assertIsNone(m.land_request)                                  # never lands by itself unless batt_autoland is switched on
        self.assertIn(("battery_land", "warn"), codes)                     # but it says so, once

    def test_warn_once_then_land_only_after_it_stays_low(self):
        m = mon(batt_autoland=True)
        for i in range(30):
            m.feed_battery(i * 0.1, 14.0)                           # 3.50 V/cell: warn
        ev = m.check(3.0, armed=True)
        self.assertEqual([e.code for e in ev], ["battery_warn"])
        self.assertEqual(m.check(3.1, True), [])                    # warned only once
        t = 3.0
        for i in range(1, 80):
            t += 0.1; m.feed_battery(t, 13.4); m.feed_link(t, 100); ev = m.check(t, True)
            if ev and ev[0].code == "battery_land": break
        self.assertEqual(m.land_request, "battery"); self.assertGreater(t, 6.0)   # filter + 3 s hold

    def test_a_short_throttle_sag_does_not_trigger_landing(self):
        m = mon()
        for i in range(20): m.feed_battery(i * 0.1, 15.2)
        m.feed_battery(2.0, 12.0); m.feed_link(2.0, 100); m.check(2.0, True)
        for i in range(10): m.feed_battery(2.1 + i * 0.1, 15.2); m.feed_link(2.1 + i * 0.1, 100); m.check(2.1 + i * 0.1, True)
        self.assertIsNone(m.land_request)

    def test_no_landing_request_when_disarmed(self):
        m = mon()
        for i in range(100): m.feed_battery(i * 0.1, 13.0); m.check(i * 0.1, armed=False)
        self.assertIsNone(m.land_request)


class TestSensorsAndLink(unittest.TestCase):
    def test_ceiling_requests_landing_once(self):
        m = mon(); m.feed_range(1.0, 3.2)
        self.assertEqual(m.check(1.0, True)[0].code, "ceiling"); self.assertEqual(m.land_request, "ceiling")
        m.feed_range(1.1, 3.3); self.assertEqual(m.check(1.1, True), [])

    def test_lost_range_while_flying_lands_but_not_on_the_ground(self):
        m = mon(); m.feed_range(1.0, 0.5); m.feed_link(2.5, 100)
        self.assertEqual(m.check(2.5, True)[0].code, "sensor_lost")
        g = mon(); g.feed_range(1.0, 0.04); g.feed_link(3.0, 100)
        self.assertEqual(g.check(3.0, True), [])

    def test_link_loss_while_armed(self):
        m = mon(); m.feed_link(1.0, 5.0)
        self.assertEqual(m.check(1.0, True), [])
        ev = m.check(2.6, True)
        self.assertEqual([e.code for e in ev], ["link_lost"]); self.assertEqual(m.land_request, "link_lost")
        self.assertEqual(mon().check(9.0, armed=False), [])


class TestLandingRamp(unittest.TestCase):
    def test_ramp(self):
        self.assertAlmostEqual(landing_throttle(0, 0.4), 0.4)
        self.assertAlmostEqual(landing_throttle(1.0, 0.4, 0.0, 2.0), 0.2)
        self.assertEqual(landing_throttle(2.5, 0.4, 0.05, 2.0), 0.05)


if __name__ == "__main__":
    unittest.main()
