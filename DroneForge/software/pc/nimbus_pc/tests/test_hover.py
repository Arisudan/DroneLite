import math
import unittest

from nimbus_pc.hover import runner
from nimbus_pc.hover.controller import HoverConfig, HoverController, HoverState, _KF3
from nimbus_pc.hover.sim import DroneSim, Sample, SimParams


class TestScenarios(unittest.TestCase):
    def check_soft_end(self, m, max_touchdown=1.0):
        self.assertFalse(m["crashed"], m)
        self.assertEqual(m["final_state"], "done", m)
        self.assertLess(m["touchdown_speed_mps"], max_touchdown, m)

    def test_nominal_hovers_and_lands(self):
        m = runner.run("nominal").metrics
        self.assertTrue(m["reached_hover"])
        self.assertLess(m["alt_err_rms_m"], 0.04); self.assertLess(m["alt_err_max_m"], 0.10)
        self.assertLess(m["xy_drift_max_m"], 0.20); self.assertLess(m["overshoot_m"], 0.15)
        self.assertLess(m["max_tilt_deg"], 8.5)
        self.check_soft_end(m, 0.7)

    def test_wrong_mass_is_learned(self):
        for name, limit in (("heavy", 0.05), ("light", 0.08)):
            m = runner.run(name).metrics
            self.assertTrue(m["reached_hover"], name)
            self.assertLess(m["alt_err_rms_m"], limit, name); self.assertLess(m["overshoot_m"], 0.25, name)
            self.check_soft_end(m, 0.9)

    def test_wind_gust_is_recovered(self):
        m = runner.run("wind").metrics
        self.assertTrue(m["reached_hover"]); self.assertLess(m["xy_drift_max_m"], 0.8); self.assertLess(m["final_xy_m"], 0.5)
        self.check_soft_end(m, 0.7)

    def test_telemetry_too_slow_refuses_to_take_off(self):
        res = runner.run("slow_telemetry")
        m = res.metrics
        self.assertFalse(m["reached_hover"]); self.assertEqual(m["final_state"], "idle")
        self.assertIn("range data too slow", m["end_reason"]); self.assertIn("telemetry ratio", m["end_reason"])

    def test_dropout_and_blackout_end_in_a_controlled_landing(self):
        for name in ("dropout", "blackout"):
            res = runner.run(name); m = res.metrics
            self.assertTrue(any("sensor_loss" in e for _, e in res.events), name)
            self.check_soft_end(m, 1.0)

    def test_wrong_sign_is_caught_by_the_sign_check(self):
        res = runner.run("sign_error"); m = res.metrics
        self.assertFalse(m["reached_hover"])
        self.assertIn("sign check failed", m["end_reason"] + " ".join(e for _, e in res.events))
        self.assertLess(m["final_xy_m"], 1.0)
        self.check_soft_end(m, 1.0)

    def test_several_seeds_all_end_safely(self):
        for seed in range(3, 8):
            for name in ("nominal", "heavy", "light"):
                m = runner.run(name, seed=seed).metrics
                self.assertTrue(m["reached_hover"], (name, seed)); self.assertFalse(m["crashed"], (name, seed))
                self.assertLess(m["alt_err_rms_m"], 0.09, (name, seed))


class TestController(unittest.TestCase):
    def samples(self, t, z=0.5, v=(0.0, 0.0)):
        return [Sample(t, "optrange", {"distance_m": z, "flow_vx": v[0], "flow_vy": v[1], "quality": 200})]

    def flying(self):
        c = HoverController()
        for i in range(60):                                   # feed 3 s of good 10 Hz data first
            if i % 2 == 0:
                c.observe(i * 0.05, self.samples(i * 0.05, 0.0))
        self.assertTrue(c.start(3.0))
        return c

    def test_idle_and_done_never_arm(self):
        c = HoverController()
        self.assertFalse(c.update(0.0, [], 0.01).arm)
        c.state = HoverState.DONE
        self.assertFalse(c.update(0.1, [], 0.01).arm)

    def test_outputs_always_inside_the_envelope(self):
        c = self.flying(); cfg = c.cfg
        t = 3.0
        for i in range(2000):
            t += 0.01
            smp = self.samples(t, 0.5 + 0.3 * math.sin(i / 50), (0.4 * math.sin(i / 30), -0.4 * math.cos(i / 40))) if i % 10 == 0 else []
            cmd = c.update(t, smp, 0.01)
            self.assertLessEqual(cmd.throttle, cfg.thr_max + 1e-9)
            lim = math.radians(cfg.tilt_max_deg) / math.radians(cfg.angle_limit_deg) + 1e-9
            self.assertLessEqual(abs(cmd.roll), lim); self.assertLessEqual(abs(cmd.pitch), lim)
            if c.state == HoverState.DONE:
                break

    def test_tilt_exceeded_disarms(self):
        c = self.flying()
        c.update(3.1, [Sample(3.1, "attitude", {"pitch_deg": 40.0, "roll_deg": 0.0, "yaw_deg": 0.0})], 0.01)
        self.assertEqual(c.state, HoverState.DONE)
        self.assertFalse(c.update(3.11, [], 0.01).arm)

    def test_low_battery_only_warns_by_default_and_lands_only_when_asked(self):
        c = self.flying(); c.state = HoverState.HOVER; c.kz.x[0] = 0.5
        c.batt_v = 13.0
        c.update(3.1, self.samples(3.1, 0.5), 0.01)
        self.assertEqual(c.state, HoverState.HOVER)                          # default: warning only, the pilot decides
        self.assertTrue(any("WARNING battery" in str(e[1]) for e in c.events))
        c = self.flying(); c.cfg.batt_autoland = True; c.state = HoverState.HOVER; c.kz.x[0] = 0.5
        c.batt_v = 13.0
        c.update(3.1, self.samples(3.1, 0.5), 0.01)
        self.assertEqual(c.state, HoverState.LAND)                           # the OPTIONAL auto-landing
        c = self.flying(); c.state = HoverState.HOVER; c.kz.x[0] = 1.5
        c.update(3.1, self.samples(3.1, 1.5), 0.01)
        self.assertEqual(c.state, HoverState.LAND)

    def test_preflight_rejects_slow_or_gappy_data_but_never_a_low_battery(self):
        c = HoverController()
        for i in range(8):                                    # 4 Hz for 2 s
            c.observe(1.0 + i * 0.25, self.samples(1.0 + i * 0.25, 0.0))
        ok, why = c.preflight(3.0)
        self.assertFalse(ok); self.assertTrue(any("too slow" in w for w in why))
        c2 = HoverController(); c2.batt_v = 13.5
        for i in range(40):
            c2.observe(1.0 + i * 0.05, self.samples(1.0 + i * 0.05, 0.0))
        ok, why = c2.preflight(3.0)
        self.assertTrue(ok, why); self.assertFalse(any("battery" in w for w in why))       # a low battery never blocks the takeoff
        self.assertIn("WARNING battery", c2.battery_warning())                             # ... it only warns

    def test_bias_filter_learns_a_constant_model_error(self):
        kf = _KF3()
        z = vz = 0.0
        for i in range(600):                                  # truth accelerates at 0.8 m/s^2 more than the model says; 10 Hz range
            dt = 0.01
            a_true = 0.8 - 0.8 * vz                           # damped so z stays bounded
            vz += a_true * dt; z += vz * dt
            kf.predict(dt, -0.8 * kf.x[1])
            if i % 10 == 0:
                kf.update(z)
        self.assertAlmostEqual(kf.x[2], 0.8, delta=0.35)
        self.assertAlmostEqual(kf.x[0], z, delta=0.03)


class TestSim(unittest.TestCase):
    def test_hover_throttle_holds_and_gap_blocks_telemetry(self):
        sim = DroneSim(SimParams(gaps=[(1.0, 2.0)]), seed=3)
        sim.z = 0.5
        arrivals = []
        for i in range(300):
            sim.step(0.01, 0.0, 0.0, 0.42, True)
            arrivals += [s.t_arrival for s in sim.poll() if s.kind == "optrange"]
        self.assertTrue(arrivals)
        self.assertFalse(any(1.0 <= t < 2.0 for t in arrivals))
        self.assertFalse(sim.crashed)

    def test_wrong_hardware_sign_reverses_the_motion(self):
        for sign in (1.0, -1.0):
            sim = DroneSim(SimParams(pitch_sign=sign), seed=1)
            sim.z = 0.5
            for _ in range(100):
                sim.step(0.01, 0.0, 0.05, 0.42, True)
            self.assertGreater(sim.vx * sign, 0.0)


class TestFlightDataHover(unittest.TestCase):
    """Hover throttle from flight data and relative voltage (incident 2026-11-06: the FC read the pack ~18 % low), jerk-limited references, landing ramp, XY integral."""

    def test_a_pack_reading_scaled_by_082_does_not_change_the_takeoff_or_hold(self):
        a = runner.run("nominal", seed=3).metrics
        b = runner.run("nominal", seed=3, params=SimParams(batt_read_scale=0.82)).metrics
        self.assertTrue(b["reached_hover"]); self.assertFalse(b["crashed"])
        self.assertLess(b["alt_err_rms_m"], 0.03); self.assertLess(b["touchdown_speed_mps"], 0.30)
        self.assertLess(abs(b["t_hover_s"] - a["t_hover_s"]), 2.0); self.assertLess(b["overshoot_m"], 0.05)

    def test_hover_throttle_scales_only_with_the_relative_voltage(self):
        ctl = HoverController(HoverConfig(thr_hover0=0.42, batt_v0=16.2, thrust_exp=1.0))
        ctl.batt_v = 13.3; ctl.v_start = 13.3
        self.assertAlmostEqual(ctl._thr_hover_now(), 0.42, places=6)                # absolute level (13.3 vs batt_v0 16.2) does not matter
        ctl.batt_v = 13.3 * 0.95
        self.assertGreater(ctl._thr_hover_now(), 0.42 + 0.03)                        # a 5 % sag does

    def test_hover_is_learned_from_flight_data_and_reported_after_5_s(self):
        cfg = HoverConfig(); sim = SimParams(hover_thr=0.46)                          # the real hover throttle differs from the configured 0.42
        ctl, s = HoverController(cfg), DroneSim(sim, 1)
        dt, learned_at = 0.01, None
        while s.t < 40.0:
            smp = s.poll()
            if ctl.state == HoverState.IDLE:
                ctl.observe(s.t, smp); smp = []
                if s.t > 3.5: ctl.start(s.t)
            cmd = ctl.update(s.t, smp, dt); s.step(dt, cmd.roll, cmd.pitch, cmd.throttle, cmd.arm)
            if learned_at is None and ctl.hover_learned() is not None: learned_at = s.t
        self.assertEqual(ctl.state, HoverState.HOVER)
        h = ctl.hover_learned()
        self.assertIsNotNone(h); self.assertGreater(h[2], 5.0)
        expect = sim.thr_idle + (0.46 - sim.thr_idle) * (sim.batt_v0 / h[1]) ** 2
        self.assertAlmostEqual(h[0], expect, delta=0.02)
        self.assertIsNotNone(ctl.hover_ref)

    def test_hover_learned_is_none_before_any_hover(self):
        self.assertIsNone(HoverController().hover_learned())

    def test_landing_descends_slowly_ramps_down_and_touches_down_softly(self):
        r = runner.run("nominal")
        m = r.metrics
        self.assertEqual(m["final_state"], "done"); self.assertLess(m["touchdown_speed_mps"], 0.30)
        land = [x for x in r.rows if x["state"] == "land"]
        self.assertTrue(land)
        self.assertLess(max(-x["vz"] for x in land if x["z"] > 0.15), 0.20)          # land_rate 0.10 m/s: never a fast descent from height
        self.assertLess(abs(land[-1]["x"] - land[0]["x"]) + abs(land[-1]["y"] - land[0]["y"]), 0.3)    # XY held
        self.assertTrue(any("landed" in e for _, e in r.events))

    def test_height_change_is_smooth_and_reaches_the_new_height(self):
        r = runner.run("nominal", duration=46, land_at=44.0, height_steps=[[20.0, 0.9]])
        rows = [x for x in r.rows if 20.0 <= x["t"] <= 44.0]
        self.assertLess(abs(rows[-1]["z"] - 0.9), 0.05)
        vz = [x["vz"] for x in rows]
        self.assertLess(max(vz), 0.40); self.assertLess(r.metrics["max_tilt_deg"], 8.0); self.assertFalse(r.metrics["crashed"])

    def test_xy_integral_removes_a_steady_push_without_oscillating(self):
        import statistics
        w = [(17.0, 100.0, 0.15, -0.15)]
        base = runner.run("nominal", duration=50, land_at=None, params=SimParams(wind=w), cfg=HoverConfig(ki_xy=0.0))
        r = runner.run("nominal", duration=50, land_at=None, params=SimParams(wind=w))
        d = lambda res, t0: [math.hypot(x["x"], x["y"]) for x in res.rows if x["t"] >= t0]
        self.assertLess(max(d(r, 20.0)), 0.3); self.assertLess(max(d(r, 20.0)), max(d(base, 20.0)))
        last = d(r, 40.0)
        self.assertLess(statistics.mean(last), 0.15); self.assertLess(statistics.pstdev(last), 0.03)      # settles near the point, no oscillation

    def test_integral_is_off_below_15_cm_and_in_landing(self):
        ctl = HoverController(); ctl.state = HoverState.HOVER; ctl.ix = [0.5, 0.5]; ctl.kz.x[0] = 0.1
        self.assertEqual(ctl._horizontal(0.01), (0.0, 0.0)); self.assertEqual(ctl.ix, [0.0, 0.0])
        ctl.kz.x[0] = 0.5; ctl.ix = [0.5, 0.5]; ctl._set_state(1.0, HoverState.LAND)
        ctl._horizontal(0.01); self.assertEqual(ctl.ix, [0.0, 0.0])


if __name__ == "__main__":
    unittest.main()


class TestSafetyScenarios(unittest.TestCase):
    def test_low_battery_makes_the_controller_land_softly_before_empty(self):
        from nimbus_pc.hover.runner import run
        r = run("low_battery", duration=90, land_at=None)
        m = r.metrics
        self.assertTrue(m["reached_hover"]); self.assertFalse(m["crashed"]); self.assertEqual(m["final_state"], "done")
        self.assertLess(m["touchdown_speed_mps"], 0.6)
        txt = " | ".join(str(e[1]) for e in r.events)
        self.assertIn("safety: battery", txt); self.assertIn("SAFETY [warn] battery low", txt)

    def test_hover_height_above_the_indoor_limit_ends_in_a_landing_not_a_crash(self):
        from nimbus_pc.hover.runner import run
        r = run("ceiling", duration=60, land_at=None)
        self.assertFalse(r.metrics["crashed"]); self.assertEqual(r.metrics["final_state"], "done")
        self.assertTrue(any("ceiling" in str(e[1]) for e in r.events))


GE = dict(ground_effect_bonus=0.25, ground_effect_h_m=0.1)      # the measured real drone: ground effect lifts it at ~0.62 throttle, it needs ~0.68 to climb


class TestRealDroneProfile(unittest.TestCase):
    """The saved flight profile against the MEASURED plant (hover 62.8 % at 15.2 V, thrust ~ throttle^2, 382 g, sensor over-read below 35 cm).
    The first real flight jumped violently with the old settings (16.2 V reference, cap 0.94, linear thrust model): these tests keep that from coming back."""

    def _run(self, hover=0.68, exp=2.0, v0=15.2, delay=(0.08, 0.15), old=False):
        from dataclasses import replace
        from nimbus_core import settings as S
        from nimbus_pc.hover import runner as R
        from nimbus_pc.hover.controller import HoverConfig
        st = S.FlightSettings(hover_height=0.5, thr_hover0=0.68, thr_max=0.80, measured=True, batt_ref_v=15.2)
        cfg = HoverConfig()
        if old:
            cfg.hover_height, cfg.thr_hover0, cfg.thr_max, cfg.batt_v0, cfg.use_lift_ref = 0.5, 0.628, 0.94, 16.2, False; cfg.lift_debounce_s, cfg.takeoff_lift_margin, cfg.dropout_low_m, cfg.dropout_step_m = 0.0, 0.030, -1.0, 9.0       # (the old controller had no lift debounce, no drop-out rejection, margin +0.03) # the settings of the violent flight
        else:
            S.apply_to_config(st, cfg)
        sim = {"hover_thr": hover, "thrust_exp": exp, "batt_v0": v0, "batt_sag_per_s": 0.005, "sensor_delay": delay, "range_low_bias_m": 0.09}
        if old: sim["hover_thr"] = 0.628                          # the violent flight's plant had no ground effect
        else: sim.update(GE)
        R.SCENARIOS["profile_test"] = ("x", sim, {k: getattr(cfg, k) for k in cfg.__dataclass_fields__})
        r = R.run("profile_test", duration=44, land_at=30)
        return r.metrics, max(x["z"] for x in r.rows)

    def test_nominal_hold_is_smooth_and_stays_near_the_target(self):
        m, zmax = self._run()
        self.assertTrue(m["reached_hover"]); self.assertFalse(m["crashed"])
        self.assertLess(zmax, 0.65); self.assertLess(m["alt_err_max_m"], 0.08); self.assertLess(m["alt_err_rms_m"], 0.04); self.assertLess(m["xy_drift_max_m"], 0.3)

    def test_takeoff_is_one_smooth_climb_with_no_hops(self):
        from nimbus_core import settings as S
        from nimbus_pc.hover import runner as R
        from nimbus_pc.hover.controller import HoverConfig
        cfg = HoverConfig(); st = S.FlightSettings(hover_height=0.5, thr_hover0=0.68, thr_max=0.80, measured=True, batt_ref_v=15.2); S.apply_to_config(st, cfg)
        p = S.plant_params(st)
        R.SCENARIOS["smooth"] = ("x", {k: getattr(p, k) for k in ("hover_thr", "thrust_exp", "batt_v0", "batt_sag_per_s", "range_low_bias_m")} | GE, {k: getattr(cfg, k) for k in cfg.__dataclass_fields__})
        rows = R.run("smooth", duration=40, land_at=32).rows
        i0 = next(i for i, x in enumerate(rows) if x["z"] > 0.02)
        run_max = 0.0; worst = 0.0; touches = 0; air = False
        for x in rows[i0:]:
            if x["state"] in ("land", "done"): break
            run_max = max(run_max, x["z"])
            if run_max < 0.40: worst = max(worst, run_max - x["z"])
            if x["z"] > 0.04: air = True
            if air and x["z"] < 0.008 and run_max < 0.40: touches += 1; air = False
        self.assertLess(worst, 0.03)                      # never falls back more than 3 cm while climbing (the old takeoff hopped 10-25 cm and touched down again)
        self.assertEqual(touches, 0)
        self.assertLess(max(x["z"] for x in rows) - 0.5, 0.12)    # overshoot at most 12 cm

    def test_hostile_corners_never_jump(self):
        for kw in (dict(hover=0.64, exp=2.3, v0=14.9), dict(hover=0.70, exp=1.7, v0=15.5, delay=(0.12, 0.20)), dict(hover=0.68, exp=2.3, v0=15.5)):
            m, zmax = self._run(**kw)
            self.assertFalse(m["crashed"], kw); self.assertLess(zmax, 0.85, kw)

    def test_the_old_violent_settings_are_caught(self):
        m, zmax = self._run(old=True, hover=0.628)
        self.assertLess(zmax, 1.0)                         # was assertGreater(1.0): the old violent flight came from the fixed lift-off margin; the velocity loop (no margin) no longer jumps even with the old settings


class TestGroundReference(unittest.TestCase):
    """Real flight 2026-11-04 13:02: the range sensor read 0.07 m ON THE GROUND, the controller took it for a lift-off, recorded the creep throttle (0.55) as the hover reference,
    sat on the ground for 20 s with a wound-up FC and flipped when it finally lifted."""

    def _cfg_and_plant(self, ground):
        from nimbus_core import settings as S
        from nimbus_pc.hover.controller import HoverConfig
        cfg = HoverConfig(); st = S.FlightSettings(hover_height=0.5, thr_hover0=0.68, thr_max=0.80, measured=True, batt_ref_v=15.2); S.apply_to_config(st, cfg)
        p = S.plant_params(st)
        sim = {k: getattr(p, k) for k in ("hover_thr", "thrust_exp", "batt_v0", "batt_sag_per_s", "range_low_bias_m")} | GE; sim["ground_range_m"] = ground
        return cfg, sim

    def test_a_ground_reading_of_7cm_is_not_a_lift_off(self):
        from nimbus_pc.hover import runner as R
        cfg, sim = self._cfg_and_plant(0.07)
        R.SCENARIOS["gr"] = ("x", sim, {k: getattr(cfg, k) for k in cfg.__dataclass_fields__})
        r = R.run("gr", duration=40, land_at=32)
        self.assertTrue(r.metrics["reached_hover"]); self.assertFalse(r.metrics["crashed"])
        txt = " | ".join(str(e[1]) for e in r.events)
        import re
        m = re.search(r"ground reference (0\.\d+) m", txt); self.assertIsNotNone(m); self.assertTrue(0.065 <= float(m.group(1)) <= 0.095)     # what the (biased) sim sensor reads on the floor
        self.assertNotIn("not plausible", txt)
        # the controller must not believe it is flying while the drone is still on the floor
        first_air = next(x["t"] for x in r.rows if x["z"] > 0.01)
        first_flying_est = next((x["t"] for x in r.rows if x["state"] == "takeoff" and x["z_est"] > 0.05), None)
        self.assertIsNotNone(first_flying_est); self.assertGreater(first_flying_est, first_air - 0.2)
        self.assertLess(r.metrics["alt_err_max_m"], 0.08)

    def test_takeoff_is_refused_when_the_ground_reading_is_absurd(self):
        from nimbus_pc.hover.controller import HoverController
        from nimbus_pc.hover.sim import Sample
        c = HoverController(); c.cfg.min_rate_hz = 1.0
        for i in range(40):
            c.observe(i * 0.1, [Sample(i * 0.1, "optrange", {"distance_m": 0.45, "flow_vx": 0.0, "flow_vy": 0.0, "quality": 100}), Sample(i * 0.1, "battery", {"voltage_v": 15.5})])
        self.assertFalse(c.start(4.0))
        self.assertTrue(any("on the ground" in w for w in c.refused))

    def test_no_lift_off_within_the_time_limit_stops_the_takeoff(self):
        from nimbus_pc.hover import runner as R
        cfg, sim = self._cfg_and_plant(0.07)
        sim["hover_thr"] = 0.95; cfg.thr_max = 0.75                       # the drone can never lift with the capped throttle
        R.SCENARIOS["nolift"] = ("x", sim, {k: getattr(cfg, k) for k in cfg.__dataclass_fields__})
        r = R.run("nolift", duration=40, land_at=None)
        self.assertEqual(r.metrics["final_state"], "done"); self.assertIn("no lift-off", r.metrics["end_reason"])


class TestInvalidRange(unittest.TestCase):
    def test_a_zero_range_reading_does_not_move_the_estimate_or_the_throttle(self):
        from nimbus_pc.hover.controller import HoverController, HoverState
        from nimbus_pc.hover.sim import Sample
        c = HoverController(); c.cfg.min_rate_hz = 1.0
        for i in range(40):
            c.observe(i * 0.1, [Sample(i * 0.1, "optrange", {"distance_m": 0.07, "flow_vx": 0.0, "flow_vy": 0.0, "quality": 100}), Sample(i * 0.1, "battery", {"voltage_v": 15.5})])
        self.assertTrue(c.start(4.0, force=True))
        t = 4.0
        for _ in range(20):
            t += 0.02; c.update(t, [], 0.02)
        z_before, thr_before = c.z, c.cmd.throttle
        t += 0.02
        c.update(t, [Sample(t, "optrange", {"distance_m": 0.0, "flow_vx": 0.0, "flow_vy": 0.0, "quality": 100})], 0.02)
        self.assertAlmostEqual(c.z, z_before, delta=0.01); self.assertAlmostEqual(c.cmd.throttle, thr_before, delta=0.02)
        self.assertEqual(c.state, HoverState.TAKEOFF)


class TestTerrainGate(unittest.TestCase):
    """Nimbus-style range handling: readings the physics cannot explain are rejected, a sustained consistent jump is absorbed into a terrain state (height unchanged)."""

    def _run(self, plant):
        from nimbus_core import settings as S
        from nimbus_pc.hover import runner as R
        from nimbus_pc.hover.controller import HoverConfig
        st = S.FlightSettings(hover_height=0.5, thr_hover0=0.68, thr_max=0.80, measured=True, batt_ref_v=15.2); cfg = HoverConfig(); S.apply_to_config(st, cfg)
        p = S.plant_params(st)
        sim = {k: getattr(p, k) for k in ("hover_thr", "thrust_exp", "batt_v0", "batt_sag_per_s", "range_low_bias_m", "ground_range_m")} | GE; sim.update(plant)
        R.SCENARIOS["tg"] = ("x", sim, {k: getattr(cfg, k) for k in cfg.__dataclass_fields__})
        r = R.run("tg", duration=44, land_at=34)
        hold = [x["z"] for x in r.rows if r.metrics["t_hover_s"] and r.metrics["t_hover_s"] + 2 <= x["t"] <= 34]
        return r, max(abs(z - 0.5) for z in hold)

    def test_one_frame_spike_is_ignored(self):
        r, err = self._run({"range_spikes": [(24.0, 24.1, 1.0)]}); self.assertLess(err, 0.08)

    def test_floor_step_down_is_absorbed_and_the_height_is_held(self):
        r, err = self._run({"floor_steps": [(24.0, 99.0, -0.20)]})
        self.assertLess(err, 0.08); self.assertTrue(any("absorbed into the terrain state" in str(e[1]) for e in r.events))

    def test_box_under_the_drone_does_not_make_it_climb_away(self):
        r, err = self._run({"floor_steps": [(24.0, 28.0, 0.25)]}); self.assertLess(err, 0.20)

    def test_a_clean_flight_never_absorbs_anything(self):
        r, err = self._run({}); self.assertFalse(any("absorbed" in str(e[1]) for e in r.events)); self.assertLess(err, 0.08)


class TestHoldsAtTheTakeOffPoint(unittest.TestCase):
    """The first GUI hold wandered 0.35 m: the sign-check pulses pushed the drone aside and the controller then re-referenced the hold point to where it ended up."""

    def test_hold_stays_near_the_take_off_point_not_where_the_sign_check_left_it(self):
        import math
        from nimbus_core import settings as S
        from nimbus_pc.hover import runner as R
        from nimbus_pc.hover.controller import HoverConfig
        for seed in (1, 2, 3):
            st = S.FlightSettings(hover_height=0.5, thr_hover0=0.68, thr_max=0.80, measured=True, batt_ref_v=15.2); cfg = HoverConfig(); S.apply_to_config(st, cfg)
            p = S.plant_params(st)
            sim = {k: getattr(p, k) for k in ("hover_thr", "thrust_exp", "batt_v0", "batt_sag_per_s", "range_low_bias_m", "ground_range_m")} | GE
            R.SCENARIOS["origin"] = ("x", sim, {k: getattr(cfg, k) for k in cfg.__dataclass_fields__})
            rows = R.run("origin", duration=60, land_at=50, seed=seed).rows
            t0 = next(x["t"] for x in rows if x["state"] == "hover")
            hold = [x for x in rows if t0 + 8 <= x["t"] <= 50]
            self.assertLess(max(math.hypot(x["x"], x["y"]) for x in hold), 0.15, seed)             # was 0.35-0.40 m
            self.assertLess(max(abs(x["z"] - 0.5) for x in hold), 0.08, seed)


class TestFcHorizontalEstimate(unittest.TestCase):
    """PC XY from the FC EKF: yaw convention found from the flow, never assumed; fallback to the PC flow filter."""

    def _ctl(self, source="fc"):
        from nimbus_pc.hover.controller import HoverConfig, HoverController
        c = HoverConfig(); c.xy_source = source
        k = HoverController(c); k.kz.x[0] = 0.5; return k

    def test_unknown_until_it_agrees_then_uses_the_fc(self):
        import math
        from nimbus_pc.hover.sim import Sample
        for sign_true in (1, -1):
            k = self._ctl()
            self.assertFalse(k.fc_xy_in_use(0.0))
            t = 0.0
            for i in range(400):
                t += 0.02
                vx, vy = 0.25 * math.sin(2 * math.pi * 0.4 * t), 0.2 * math.cos(2 * math.pi * 0.3 * t)
                bx, by = k._to_body(vx, vy, 50.0, sign_true)
                k._flow_b = (t, bx, by, True)
                k.fc_xy_t = -1e9 if k.fc_xy_sign == 0 else k.fc_xy_t
                k._ingest_fcxy(t, {"vx_mps": vx, "vy_mps": vy, "yaw_deg": 50.0, "valid": True})
            self.assertEqual(k.fc_xy_sign, sign_true)
            self.assertTrue(k.fc_xy_in_use(t))
            self.assertFalse(k.fc_xy_in_use(t + 1.0))                        # stale FC frames: back to the PC flow filter

    def test_garbage_fc_data_is_never_trusted(self):
        import random
        k = self._ctl(); rng = random.Random(3); t = 0.0
        for i in range(600):
            t += 0.02
            k._flow_b = (t, 0.3 * math.sin(t), 0.2 * math.cos(1.3 * t), True)
            k._ingest_fcxy(t, {"vx_mps": rng.uniform(-0.4, 0.4), "vy_mps": rng.uniform(-0.4, 0.4), "yaw_deg": 20.0, "valid": True})
        self.assertEqual(k.fc_xy_sign, 0)

    def test_pc_source_ignores_the_fc(self):
        k = self._ctl("pc"); k._flow_b = (1.0, 0.3, 0.1, True)
        k._ingest_fcxy(1.0, {"vx_mps": 0.3, "vy_mps": 0.1, "yaw_deg": 0.0, "valid": True})
        self.assertEqual(k.fc_xy_sign, 0); self.assertFalse(k.fc_xy_in_use(1.0))

    def test_samples_reach_the_filter_through_update(self):
        import math
        k = self._ctl(); k.state = HoverState.HOVER; k.lifted = True
        t = 10.0
        for i in range(300):
            t += 0.02
            vx, vy = 0.25 * math.sin(2 * math.pi * 0.4 * t), 0.2 * math.cos(2 * math.pi * 0.3 * t)
            bx, by = k._to_body(vx, vy, 30.0, 1)
            k.update(t, [Sample(t, "optrange", {"distance_m": 0.5, "flow_vx": bx, "flow_vy": by, "quality": 150}),
                         Sample(t, "fcxy", {"vx_mps": vx, "vy_mps": vy, "yaw_deg": 30.0, "valid": True})], 0.02)
        self.assertEqual(k.fc_xy_sign, 1); self.assertEqual(k.xy_active, "fc")
        self.assertTrue(any("agrees with the flow" in e[1] for e in k.events))
