"""Flight modes (stabilize / alt_hold / pos_hold) in the hover simulator on the real-drone plant (ground effect)."""
import statistics
import unittest

from nimbus_core import settings as S
from nimbus_pc.hover import runner as R
from nimbus_pc.hover.controller import HoverConfig, HoverController

GE = dict(ground_effect_bonus=0.25, ground_effect_h_m=0.1)
H = 0.5


def fly(mode_steps=None, seed=2, duration=40, land_at=None):
    st = S.FlightSettings(hover_height=H, thr_hover0=0.68, thr_max=0.80, measured=True, batt_ref_v=15.2)
    cfg = HoverConfig(); S.apply_to_config(st, cfg); cfg.sign_check = False
    p = S.plant_params(st)
    sim = {k: getattr(p, k) for k in ("hover_thr", "thrust_exp", "batt_v0", "batt_sag_per_s", "range_low_bias_m", "ground_range_m")} | GE
    R.SCENARIOS["modes"] = ("x", sim, {k: getattr(cfg, k) for k in cfg.__dataclass_fields__})
    return R.run("modes", duration=duration, land_at=land_at, seed=seed, mode_steps=mode_steps)


def hover_t0(res):
    return next(x["t"] for x in res.rows if x["state"] == "hover")


def max_steps(rows):
    """Largest throttle step and tilt-command step between consecutive 100 ms rows."""
    dthr = max(abs(b["thr"] - a["thr"]) for a, b in zip(rows, rows[1:]))
    dtilt = max(abs(b["cmd_tilt_deg"] - a["cmd_tilt_deg"]) for a, b in zip(rows, rows[1:]))
    return dthr, dtilt


class TestModes(unittest.TestCase):
    def test_invalid_mode_refused_and_default(self):
        c = HoverController()
        self.assertEqual(c.flight_mode, "pos_hold")
        self.assertFalse(c.set_flight_mode("acro")); self.assertEqual(c.flight_mode, "pos_hold")
        for m in ("stabilize", "alt_hold", "pos_hold"):
            self.assertTrue(c.set_flight_mode(m)); self.assertEqual(c.flight_mode, m)

    def test_alt_hold_holds_height_with_zero_tilt(self):
        for seed in (1, 2, 3):
            res = fly([[0.0, "alt_hold"]], seed=seed, duration=30)
            t0 = hover_t0(res)
            rows = [x for x in res.rows if t0 + 3.0 <= x["t"] <= t0 + 13.0]
            self.assertGreater(len(rows), 90)
            self.assertEqual(max(x["cmd_tilt_deg"] for x in rows), 0.0)
            self.assertLessEqual(max(abs(x["z"] - H) for x in rows), 0.08, seed)

    def test_stabilize_constant_throttle_no_tilt_no_crash(self):
        """Entered after 9 s of settled hover (the learned hover base is then what is frozen). Open loop vertically on purpose: the height drifts, the drone must not crash in 10 s."""
        for seed in (1, 2, 3):
            t0 = hover_t0(fly([], seed=seed, duration=12))
            res = fly([[t0 + 9.0, "stabilize"]], seed=seed, duration=45)
            rows = [x for x in res.rows if t0 + 9.0 + 0.5 <= x["t"] <= t0 + 9.0 + 10.0]
            self.assertGreater(len(rows), 90)
            self.assertEqual(max(x["cmd_tilt_deg"] for x in rows), 0.0)
            self.assertLess(statistics.pstdev(x["thr"] for x in rows), 0.005, seed)
            self.assertFalse(res.metrics["crashed"], seed)

    def test_pos_hold_default_unchanged(self):
        res = fly(None, duration=30)
        t0 = hover_t0(res)
        rows = [x for x in res.rows if t0 + 3.0 <= x["t"] <= t0 + 15.0]
        self.assertLessEqual(max(abs(x["z"] - H) for x in rows), 0.08)
        self.assertNotIn("flight mode active", " ".join(str(e) for _, e in res.events))

    def test_switching_sequence_is_bumpless(self):
        res = fly(None, duration=0.1)  # warm module import only
        for seed in (1, 2):
            probe = fly([], seed=seed, duration=30)
            t0 = hover_t0(probe)
            seq = [[t0 + 3, "alt_hold"], [t0 + 9, "stabilize"], [t0 + 11, "alt_hold"], [t0 + 15, "pos_hold"], [t0 + 19, "stabilize"], [t0 + 21, "pos_hold"]]
            times = [ts for ts, _m in seq]
            res = fly(seq, seed=seed, duration=40)
            for ts in times:
                rows = [x for x in res.rows if ts <= x["t"] <= ts + 2.5 and x["state"] == "hover"]
                dthr, dtilt = max_steps(rows)
                self.assertLess(dthr, 0.03, (seed, ts, dthr)); self.assertLess(dtilt, 1.0, (seed, ts, dtilt))
            rows = [x for x in res.rows if x["t"] >= t0 + 2.0 and x["state"] == "hover"]
            self.assertFalse(res.metrics["crashed"])
            self.assertGreater(min(x["z"] for x in rows), 0.08)
            self.assertLess(max(abs(x["z"] - H) for x in rows if x["t"] > t0 + 21 + 4), 0.12)

    def test_land_works_from_stabilize(self):
        res = fly([[0.0, "stabilize"]], duration=45, land_at=20)
        self.assertFalse(res.metrics["crashed"])
        self.assertEqual(res.metrics["final_state"], "done")


if __name__ == "__main__":
    unittest.main()
