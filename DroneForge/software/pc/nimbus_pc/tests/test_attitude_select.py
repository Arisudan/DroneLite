"""Attitude from the fresher of the ATTITUDE frame and the D4 EKF frame: selection logic and a replay with the measured rates (D4 7.5 Hz / age 46 ms, attitude 1.2 Hz / age 409 ms)."""
import math
import unittest

from nimbus_core.attitude_select import AttitudeSelector, usable_d4_attitude

GOOD = {"roll_rad": 0.01, "pitch_rad": -0.02, "yaw_rad": 1.0, "valid": True, "x_m": 0, "y_m": 0, "z_m": 0}
ATT = {"roll_rad": 0.02, "pitch_rad": 0.03, "yaw_rad": 0.5}


class TestSelection(unittest.TestCase):
    def test_valid_d4_is_published_as_attitude_in_the_same_units(self):
        s = AttitudeSelector()
        out = s.feed("kinematic", GOOD, 1.0)
        self.assertEqual([n for n, _ in out], ["kinematic", "attitude"])
        a = out[1][1]
        self.assertEqual((a["roll_rad"], a["pitch_rad"], a["yaw_rad"], a["source"]), (0.01, -0.02, 1.0, "d4"))

    def test_invalid_flag_and_sentinels_are_ignored(self):
        s = AttitudeSelector()
        self.assertEqual([n for n, _ in s.feed("kinematic", dict(GOOD, valid=False), 1.0)], ["kinematic"])
        for bad in (3.2767, -3.2768, float("nan"), float("inf"), 4.0):
            self.assertIsNone(usable_d4_attitude(dict(GOOD, roll_rad=bad)), bad)
            self.assertIsNone(usable_d4_attitude(dict(GOOD, yaw_rad=bad)), bad)
        self.assertIsNone(s.t_d4)

    def test_attitude_frame_passes_through_and_is_the_fallback(self):
        ev = []
        s = AttitudeSelector(event=ev.append)
        s.feed("kinematic", GOOD, 1.0)
        out = s.feed("attitude", ATT, 2.0)                    # D4 is 1 s old (> 0.5 s): the attitude frame is the source
        self.assertEqual(out, [("attitude", ATT)])
        self.assertEqual(s.source, "att")
        self.assertTrue(any("D4" in e for e in ev) and any("attitude frame" in e for e in ev))

    def test_d4_wins_again_when_it_returns(self):
        s = AttitudeSelector()
        s.feed("attitude", ATT, 1.0)
        s.feed("kinematic", GOOD, 1.05)
        self.assertEqual(s.source, "d4")
        self.assertEqual(s.last_time(), 1.05)

    def test_other_telemetry_untouched(self):
        self.assertEqual(AttitudeSelector().feed("battery", {"voltage_v": 14.0}, 1.0), [("battery", {"voltage_v": 14.0})])


class TestReplayWithMeasuredRates(unittest.TestCase):
    def _ages(self, with_d4):
        # arrival times: D4 every 1/7.5 s, ATTITUDE every 1/1.2 s; the age of a frame when it arrives (measured medians 46 ms / 409 ms) is part of the arrival delay
        s = AttitudeSelector()
        ev = sorted([(i / 7.5 + 0.046, "kinematic") for i in range(1, 300)] + [(i / 1.2 + 0.409, "attitude") for i in range(1, 50)])
        ev = [e for e in ev if with_d4 or e[1] == "attitude"]
        ages, k, t = [], 0, 3.0
        while t < 38.0:
            while k < len(ev) and ev[k][0] <= t:
                s.feed(ev[k][1], GOOD if ev[k][1] == "kinematic" else ATT, ev[k][0]); k += 1
            ages.append(t - s.last_time())
            t += 0.01
        ages.sort()
        return ages[len(ages) // 2]

    def test_effective_attitude_age_drops_below_0p15_s(self):
        self.assertLess(self._ages(True), 0.15)
        self.assertGreater(self._ages(False), 0.3)            # attitude frame alone: the problem this fixes


if __name__ == "__main__":
    unittest.main()
