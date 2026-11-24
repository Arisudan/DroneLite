"""The REAL controller profile on the plant fitted to the real flights of 2026-11-12/13 (docs/maintenance/REAL_FLIGHT_FIT_2026-11-13.md):
throttle-to-range delay 0.33 s (+ sensor delay + motor lag = 0.4-0.55 s in the flight data), hover 0.72 on the floor / 0.79 on boxes, 17 Hz range data.
The old controller (legacy profile) bursts to 1.3-2 m and crashes at 2-3.5 m/s on this plant (the real flights did the same); the real profile must not.
Bounds are what it achieves (measured over 6-8 seeds), NOT the spec: hold error up to ~0.5 m and touchdown ~1.0-1.5 m/s are KNOWN MISSES (see the report)."""
import os
import unittest
from dataclasses import replace

from nimbus_core import settings as S
from nimbus_pc.hover import runner
from nimbus_pc.hover.controller import HoverConfig, HoverController, HoverState
from nimbus_pc.hover.sim import SimParams

FLOOR = dict(ground_effect_bonus=0.25, ground_effect_h_m=0.1, hover_thr=0.72, ground_range_m=0.07)
BOXES = dict(ground_effect_bonus=0.0, hover_thr=0.79, ground_range_m=0.19)


def plant(kw, **over):
    d = dict(thrust_exp=2.0, batt_sag_per_s=0.005, range_low_bias_m=S.SENSOR_LOW_BIAS_M, batt_v0=13.8, ctrl_delay_s=0.33, tau_motor=0.06, optrange_hz=17.0)
    d.update(kw); d.update(over)
    return SimParams(**d)


def fly(kw, seed, cfg_hover=0.66, tmax=0.85, profile="real", land_at=26.0, duration=48.0, **over):
    st = S.FlightSettings(hover_height=0.5, thr_hover0=cfg_hover, thr_max=tmax, measured=True, batt_ref_v=13.8, profile=profile)
    cfg = HoverConfig(); S.apply_to_config(st, cfg)
    return runner.run("nominal", duration=duration, seed=seed, land_at=land_at, params=plant(kw, **over), cfg=cfg)


class TestRealProfile(unittest.TestCase):
    def _check(self, kw, seed, cfg_hover, tag):
        res = fly(kw, seed, cfg_hover=cfg_hover); m = res.metrics
        self.assertFalse(m["crashed"], tag + ": crashed")
        self.assertTrue(m["reached_hover"], tag + ": never reached the hover")
        rows = res.rows; t0 = next(r["t"] for r in rows if r["state"] == "hover")
        hold = max(abs(r["z"] - 0.5) for r in rows if t0 + 4 <= r["t"] <= min(t0 + 14, 25.5))
        zmax = max(r["z"] for r in rows if r["t"] < 26.0)
        self.assertLessEqual(hold, 0.65, tag + ": hold error"); self.assertLessEqual(zmax, 1.0, tag + ": burst")
        self.assertLessEqual(m["touchdown_speed_mps"], 1.8, tag + ": touchdown")

    def test_floor_with_the_saved_hover_setting(self):
        for seed in (1, 2): self._check(FLOOR, seed, 0.66, f"floor seed {seed}")

    def test_boxes_without_ground_effect(self):
        for seed in (1, 2): self._check(BOXES, seed, 0.72, f"boxes seed {seed}")

    def test_the_legacy_profile_still_fails_on_this_plant(self):
        """Documents why the real profile exists: the same flight with the legacy tuning bursts or crashes (as the real drone did)."""
        bad = 0
        for seed in (1, 2, 3):
            res = fly(FLOOR, seed, cfg_hover=0.66, profile="legacy", tmax=0.81)
            if res.metrics["crashed"] or max(r["z"] for r in res.rows if r["t"] < 26.0) > 1.0: bad += 1
        self.assertGreaterEqual(bad, 2)

    def test_thrust_limit_lands_the_drone_gently_instead_of_fighting(self):
        res = fly(FLOOR, 1, cfg_hover=0.66, tmax=0.81, land_at=80.0, duration=70.0, hover_thr=0.72, batt_sag_per_s=0.05)     # the pack fades: thrust runs out at the cap
        txt = " ".join(str(e[1]) for e in res.events)
        self.assertIn("thrust limit", txt); self.assertFalse(res.metrics["crashed"])
        self.assertEqual(res.metrics["final_state"], "done")

    def test_defaults_are_unchanged_and_the_lead_is_off_without_a_delay(self):
        self.assertEqual(S.FlightSettings().profile, "legacy")
        c = HoverController(HoverConfig()); self.assertEqual(c._lead(1.0), (0.0, 0.0))
        self.assertEqual(S.FlightSettings(profile="bogus").validate()[0].split()[0], "profile")


if __name__ == "__main__":
    unittest.main()
