"""bfl.hold_report: what the FC log says about the sideways hold once the firmware logs debug_mode DF_HOLD (v4.7.4R): EKF speed and the VO aid's presence / gaps."""
import unittest

from nimbus_gcs import bfl

NOAID = -32768


class _Log:
    def __init__(self, cols):
        self.cols = cols; self.ix = {k: i for i, k in enumerate(cols)}
    def column(self, k): return self.cols[k]


def _make(n=1000, hz=100.0, aid=lambda k: None, vel=lambda k: (0, 0), floor=5, up_at=100):
    cols = {"surfaceRaw": [floor if k < up_at else floor + 55 for k in range(n)]}
    v = [vel(k) for k in range(n)]; a = [aid(k) for k in range(n)]
    cols["debug[0]"] = [x[0] for x in v]; cols["debug[1]"] = [x[1] for x in v]
    cols["debug[2]"] = [NOAID if x is None else x[0] for x in a]; cols["debug[3]"] = [NOAID if x is None else x[1] for x in a]
    return _Log(cols), [k / hz for k in range(n)]


class TestHoldReport(unittest.TestCase):
    def test_aid_presence_gaps_and_speed(self):
        aid = lambda k: (30, -40) if (300 <= k < 500 or 600 <= k < 800) else None             # present 400 of the 900 airborne frames; gaps 2.0 s, 1.0 s, 2.0 s
        lg, rel = _make(aid=aid, vel=lambda k: (100, 0))
        lines = bfl.hold_report(lg, 0, rel)
        self.assertEqual(len(lines), 2); self.assertIn("9.0 s airborne", lines[0]); self.assertIn("EKF speed rms 0.100 m/s", lines[0])
        self.assertIn("present in 44 %", lines[1]); self.assertIn("longest gap without it 2.0 s", lines[1]); self.assertIn("rms 0.050 m", lines[1])           # sqrt(0.03^2 + 0.04^2) = 0.05 m

    def test_no_aid_at_all_says_so_and_names_the_causes(self):
        lg, rel = _make(); lines = bfl.hold_report(lg, 0, rel)
        self.assertIn("none reached the hold", lines[1]); self.assertIn("ALT HOLD", lines[1]); self.assertIn("radio link", lines[1])

    def test_a_flight_that_never_left_the_floor_gives_no_report(self):
        lg, rel = _make(floor=5, up_at=10_000); self.assertEqual(bfl.hold_report(lg, 0, rel), [])

    def test_always_present_aid(self):
        lg, rel = _make(aid=lambda k: (10, 0)); lines = bfl.hold_report(lg, 0, rel); self.assertIn("present in 100 %", lines[1]); self.assertIn("longest gap without it 0.0 s", lines[1])


if __name__ == "__main__":
    unittest.main()
