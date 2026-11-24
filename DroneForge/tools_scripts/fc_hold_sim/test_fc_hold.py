"""Tests of the FC-side height hold (firmware flight/height_hold_core.c, compiled on the PC) on the plant fitted to the real flights. Run: python3 -m unittest test_fc_hold -v (in this folder)."""
import os, subprocess, sys, unittest
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
subprocess.run([os.path.join(HERE, "build.sh")], check=True, capture_output=True)
from fc_hold_sim import *
import ctypes
ctypes.CDLL(os.path.join(HERE, "libhh.so")).hh_cap_landing.restype = ctypes.c_int


def run(kw, ref=0.66, seed=1, **fk):
    core = FcCore(); return core, fly(Plant(seed=seed, **kw), core, f_hover_ref=ref, **fk)


class TestFcHold(unittest.TestCase):
    def test_nominal_floor_flight_takes_off_holds_and_lands(self):
        for seed in (1, 2, 3):
            _, r = run(dict(hover=0.72), seed=seed)
            self.assertFalse(r["crashed"] or r["fail"]); self.assertTrue(r["landed"])
            self.assertLessEqual(r["hold_max"], 0.30); self.assertLessEqual(r["zmax"], 1.0); self.assertLessEqual(r["vzmax"], 1.0)     # KNOWN MISS: the ground-effect margin (0.04) overshoots up to ~0.4 m on plants with little ground effect

    def test_take_off_works_when_the_pcs_hover_reference_is_far_too_low(self):
        """real flights: lift-off at 0.78 while the saved hover reference was 0.66 (the old 1.15x cap stopped the take-off on the floor)"""
        for kw in (dict(hover=0.78), dict(hover=0.79, ge=0.0, ground_range=0.19)):
            _, r = run(kw, ref=0.66); self.assertFalse(r["fail"] or r["crashed"], str(kw)); self.assertTrue(r["landed"])

    def test_a_drone_held_down_near_the_floor_still_lifts_without_a_burst(self):
        _, r = run(dict(hover=0.69, tether_g=0.08, tether_h=0.25))
        self.assertFalse(r["fail"] or r["crashed"]); self.assertLessEqual(r["zmax"], 1.3); self.assertLessEqual(r["vzmax"], 1.2)     # KNOWN MISS: overshoot ~0.6 m when something held the drone down and lets go

    def test_robust_to_the_delays_of_the_fc_side_signal_path(self):
        """sensor age and command-path delay up to 0.10 s / 0.08 s (the radio path of the PC loop, 0.4-0.55 s, is not in this loop): no crash, no fly-away"""
        for sd, ad in ((0.04, 0.03), (0.10, 0.08)):
            for hover in (0.64, 0.72, 0.78):
                _, r = run(dict(hover=hover, sensor_delay=sd, act_delay=ad))
                self.assertFalse(r["crashed"] or r["fail"], f"{hover} {sd} {ad}"); self.assertTrue(r["landed"]); self.assertLessEqual(r["zmax"], 0.95)

    def test_thrust_limit_lands_instead_of_fighting(self):
        core = FcCore(); pl = Plant(hover=0.70, sag=0.025)         # the pack fades slowly: it lifts, then after a while the cap is not enough
        r = fly(pl, core, f_hover_ref=0.66, hold_s=150.0, t_max=200.0, land=False)
        self.assertFalse(r["crashed"]); self.assertIn(2, r["trace"]["mode"])
        self.assertTrue(ctypes.CDLL(os.path.join(HERE, "libhh.so")).hh_cap_landing() == 1 or r["landed"])

    def test_the_real_drone_case_lifts_out_of_ground_effect_and_reaches_the_target(self):
        """11-13 15:00 tethered run: lift-off at 0.60 with a free-air hover of 0.69 (ground effect over the first ~0.2 m), FC speed bias +0.2 m/s near the floor, target 0.3 m.
        The first version stalled at 0.1-0.2 m (base = the lift-off throttle, integral capped at +-0.12) and the drone sank back."""
        for bias in (0.0, 0.2):
            for sd, ad in ((0.05, 0.04), (0.10, 0.08)):
                for seed in (1, 2, 3):
                    _, r = run(dict(hover=0.69, ge=0.35, ge_h=0.2, vz_bias=bias, sensor_delay=sd, act_delay=ad), seed=seed, target=0.3, hold_s=14.0)
                    tag = f"bias {bias} delays {sd}/{ad} seed {seed}"
                    self.assertFalse(r["crashed"] or r["fail"], tag); self.assertTrue(r["landed"], tag)
                    self.assertGreaterEqual(r["zmax"], 0.24, tag + ": never reached the target"); self.assertLessEqual(r["zmax"], 0.8, tag); self.assertLessEqual(r["vzmax"], 1.7, tag)       # v4.5 rev2: kv_z 0.054 -> 0.10 (1.4 -> 1.7 m/s surge on this harshest ground-effect plant, target 0.3 m = hand-over inside the GE band; <=0.5 m targets are above it). KNOWN MISS: up to ~0.4 m overshoot on a 0.3 m target when the drone leaves the ground-effect cushion

    def test_height_step_is_smooth(self):
        _, r = run(dict(hover=0.72), step_to=0.9, hold_s=24.0)
        self.assertFalse(r["crashed"]); self.assertLessEqual(r["zmax"], 1.2); self.assertLessEqual(r["vzmax"], 0.9)


if __name__ == "__main__":
    unittest.main()


class TestSidewaysLoop(unittest.TestCase):
    """FC sideways loop (firmware flight/xy_hold_core.c) on a simulated horizontal plant: ANGLE-mode lag, constant push (a level error), 0.08 s speed delay, noise."""
    def setUp(self):
        import xy_sim
        self.xy = xy_sim

    def test_without_the_loop_a_level_error_drifts_away(self):
        self.assertGreater(self.xy.run(loop=False)["max_dist"], 5.0)

    def test_the_loop_holds_against_a_constant_push_and_learns_both_signs(self):
        for push in ((0.14, 0.0), (0.0, 0.14), (0.10, -0.10)):
            for seed in (1, 2, 3):
                r = self.xy.run(push=push, seed=seed)
                self.assertFalse(r["disabled"], str(push)); self.assertEqual(r["sign"], (1.0, 1.0))
                self.assertLessEqual(r["max_dist"], 1.2, str(push)); self.assertLessEqual(r["end_dist"], 0.5, str(push)); self.assertLessEqual(r["max_speed"], 0.8)

    def test_a_wrong_axis_sign_is_found_and_inverted_by_the_self_test(self):
        for sm, want in (((1.0, -1.0), (1.0, -1.0)), ((-1.0, 1.0), (-1.0, 1.0)), ((-1.0, -1.0), (-1.0, -1.0))):
            r = self.xy.run(sign_meas=sm, push=(0.10, 0.10), over={'skip_test': 0})
            self.assertEqual(r["sign"], want); self.assertFalse(r["disabled"]); self.assertLessEqual(r["end_dist"], 0.6); self.assertLessEqual(r["max_dist"], 1.2)

    def test_an_axis_that_does_not_respond_ends_safe(self):
        r = self.xy.run(resp=(1.0, 0.0), push=(0.10, 0.10))
        self.assertLessEqual(r["max_speed"], 1.2)          # never a runaway: the guard switches the loop off (it cannot hold a dead axis)

    def test_the_runaway_guard_switches_the_loop_off(self):
        r = self.xy.run(push=(0.9, 0.0), t_end=15.0)        # a push no 4 degree tilt can hold
        self.assertTrue(r["disabled"] or r["max_speed"] < 1.5)


class TestFrozenRangeDetector(unittest.TestCase):
    """Nimbus 'flatline': the whole range payload identical for 1.5 s while flying = frozen sensor."""
    def setUp(self):
        self.L = ctypes.CDLL(os.path.join(HERE, "libhh.so"))
        self.L.fl_update.argtypes = [ctypes.c_float, ctypes.c_uint]; self.L.fl_update.restype = ctypes.c_int
        self.L.fl_sig.argtypes = [ctypes.c_int, ctypes.c_uint, ctypes.c_uint, ctypes.c_int, ctypes.c_int, ctypes.c_uint]; self.L.fl_sig.restype = ctypes.c_uint
        self.L.fl_init()

    def test_a_live_payload_never_trips_it(self):
        for i in range(600): self.assertFalse(self.L.fl_update(0.02, self.L.fl_sig(523 + (i % 3), 180, 3, i % 5 - 2, 1, 90)))

    def test_a_frozen_payload_trips_after_1_5_s(self):
        sig = self.L.fl_sig(523, 180, 3, 0, 0, 90); trip = None
        for i in range(200):
            if self.L.fl_update(0.02, sig): trip = i * 0.02; break
        self.assertIsNotNone(trip); self.assertAlmostEqual(trip, 1.5, delta=0.1)

    def test_any_changing_field_resets_it(self):
        base = (523, 180, 3, 0, 0, 90)
        for field in range(6):
            self.L.fl_init(); tripped = False
            for i in range(300):
                v = list(base)
                if i % 40 == 39: v[field] += 1                  # one field changes every 0.8 s: never frozen
                tripped = tripped or bool(self.L.fl_update(0.02, self.L.fl_sig(*v)))
            self.assertFalse(tripped, f"field {field}")


def test_floor_integrator_cap():
    """v4.3: on the floor with the FC take-off running, roll/pitch I is limited to +-cap instead of zeroed; cap 0 keeps the old zero."""
    import ctypes
    L = ctypes.CDLL(os.path.join(os.path.dirname(os.path.abspath(__file__)), "libhh.so"))
    L.gh_cap.restype = ctypes.c_float; L.gh_cap.argtypes = [ctypes.c_float, ctypes.c_float]
    assert L.gh_cap(250.0, 100.0) == 100.0 and L.gh_cap(-300.0, 100.0) == -100.0
    assert L.gh_cap(60.0, 100.0) == 60.0 and L.gh_cap(-60.0, 100.0) == -60.0       # the ~55-70 the clean creeps settled at passes untouched
    assert L.gh_cap(60.0, 0.0) == 0.0                                                # cap 0 = hold at zero (v4.1)
    assert L.gh_cap(float("nan"), 100.0) == 0.0 and L.gh_cap(float("inf"), 100.0) == 0.0


def test_untested_sideways_axis_damps_with_full_authority():
    """v4.4: before the sign test has run, an axis damps its speed with the nominal sign up to max_deg (4 deg), not +-1.5 deg"""
    import ctypes
    L = ctypes.CDLL(os.path.join(os.path.dirname(os.path.abspath(__file__)), "libhh.so"))
    L.xy_update.argtypes = [ctypes.c_float] * 4 + [ctypes.c_int, ctypes.POINTER(ctypes.c_float)]
    L.xy_params.restype = ctypes.POINTER(ctypes.c_float)
    L.xy_defaults(); L.xy_params()[9] = 0.01; L.xy_params()[14] = 0.0   # calm_speed so low that the sign test never starts (skip_test off)
    out = (ctypes.c_float * 2)()
    for v, want in ((0.3, -1.5), (0.6, -3.0), (0.75, -3.75)):
        for _ in range(30): L.xy_update(ctypes.c_float(0.02), ctypes.c_float(0.0), ctypes.c_float(v), ctypes.c_float(0.2), 1, out)
        assert abs(out[0] - want) < 0.05, (v, out[0])


def _handover_step(lib):
    """early 'lifted' detection (z floats at 0.08 m on the floor, vz +0.1), creep up to ~0.73, then the height reaches the hand-over level: returns (throttle before, throttle after, mode after)"""
    import ctypes
    L = ctypes.CDLL(lib)
    for n in ("hh_update",): getattr(L, n).restype = ctypes.c_float
    L.hh_takeoff.argtypes = [ctypes.c_float] * 3; L.hh_update.argtypes = [ctypes.c_float] * 4
    L.hh_defaults(); L.hh_takeoff(ctypes.c_float(0.66), ctypes.c_float(0.5), ctypes.c_float(13.5))
    f = 0.0; t = 0.0
    while t < 120.0:
        if t < 1.0: z, vz = 0.0, 0.0
        elif f < 0.73: z, vz = 0.08, (0.1 if f < 0.52 else 0.0)
        else: break
        f = L.hh_update(ctypes.c_float(0.02), ctypes.c_float(z), ctypes.c_float(vz), ctypes.c_float(13.5)); t += 0.02
    before = f
    out = []
    for z, vz in ((0.50, 0.2),) + ((0.50, 0.15),) * 4:
        out.append(L.hh_update(ctypes.c_float(0.02), ctypes.c_float(z), ctypes.c_float(vz), ctypes.c_float(13.5)))
    return before, out[0], L.hh_mode()


def test_handover_does_not_step_the_throttle_down():
    """v4.5: with an early (low) lift-off detection the old code left base + i_max far below the creep throttle: the output fell by >0.1 at the hand-over"""
    here = os.path.dirname(os.path.abspath(__file__))
    before, after, mode = _handover_step(os.path.join(here, "libhh.so"))
    assert mode == 0, "the hand-over must have happened"
    assert after >= before - 0.03, (before, after)



def test_no_self_test_means_no_deliberate_lean():
    """v4.5 rev2: with skip_test the loop never tilts the drone on purpose: a calm drone gets (almost) no output, and the nominal sign is active at once"""
    import ctypes
    L = ctypes.CDLL(os.path.join(os.path.dirname(os.path.abspath(__file__)), "libhh.so"))
    L.xy_update.argtypes = [ctypes.c_float] * 4 + [ctypes.c_int, ctypes.POINTER(ctypes.c_float)]
    L.xy_defaults(); out = (ctypes.c_float * 2)(); worst = 0.0
    for _ in range(300): L.xy_update(ctypes.c_float(0.02), ctypes.c_float(0.0), ctypes.c_float(0.0), ctypes.c_float(0.2), 1, out); worst = max(worst, abs(out[0]), abs(out[1]))
    assert worst < 0.01, worst                       # the old test leaned 2.5 deg for 0.8 s here
    for _ in range(40): L.xy_update(ctypes.c_float(0.02), ctypes.c_float(0.0), ctypes.c_float(0.4), ctypes.c_float(0.2), 1, out)
    assert out[0] < -1.0                             # a speed to the right is opposed at once (negative roll)


class TestFastLiftOff(unittest.TestCase):
    def test_hand_over_comes_within_14_s_without_a_big_overshoot(self):
        """v4.5 rev3: fast creep (0.03/s from 0.82 x the hover reference) until 8 cm up, then the slow creep: the slow creep alone needed 24-36 s"""
        for kw in (dict(hover=0.66), dict(hover=0.72), dict(hover=0.78), dict(hover=0.80), dict(hover=0.72, ge=0.45, ge_h=0.4), dict(hover=0.69, ge=0.35, ge_h=0.2)):
            for seed in (1, 2):
                r, tt = _handover_time(kw, seed)
                self.assertFalse(r["crashed"] or r["fail"], (kw, seed))
                self.assertLessEqual(tt, 14.0, (kw, seed, tt)); self.assertLessEqual(r["zmax"], 0.68, (kw, seed)); self.assertLessEqual(r["vzmax"], 0.65, (kw, seed))
                self.assertLessEqual(r["hold_rms"], 0.03, (kw, seed))


def _handover_time(kw, seed):
    core = FcCore(); r = fly(Plant(seed=seed, **kw), core, f_hover_ref=0.66, target=0.5)
    tr = r["trace"]; th = next((i for i in range(len(tr["t"])) if tr["mode"][i] == 0), None)
    return r, (tr["t"][th] if th is not None else 99.0)


class TestXyV47ST(unittest.TestCase):
    """v4.7 ST (2026-11-21): stick-through, brake-then-hold, trim kept, climb trim, PC visual-odometry position aid."""
    def setUp(self):
        L = ctypes.CDLL(os.path.join(HERE, "libhh.so"))
        if not hasattr(L, "xy_update_st"): self.skipTest("libhh.so is older than v4.7")
        L.xy_update_st.argtypes = [ctypes.c_float] * 4 + [ctypes.c_int, ctypes.c_float, ctypes.c_float, ctypes.c_int, ctypes.c_float, ctypes.c_float, ctypes.POINTER(ctypes.c_float)]
        L.xy_update.argtypes = [ctypes.c_float] * 4 + [ctypes.c_int, ctypes.POINTER(ctypes.c_float)]
        L.xy_p.restype = ctypes.c_float; L.xy_i.restype = ctypes.c_float
        L.xy_defaults(); self.L = L; self.out = (ctypes.c_float * 2)()

    def st(self, v=(0.0, 0.0), hold=1, stick=(0.0, 0.0), aid=None, n=1):
        for _ in range(n):
            self.L.xy_update_st(0.02, v[0], v[1], 0.8, hold, stick[0], stick[1], aid is not None, aid[0] if aid else 0.0, aid[1] if aid else 0.0, self.out)
        return self.out[1], self.out[0]          # (pitch, roll) deg

    def test_a_deflected_stick_gets_no_hold_tilt(self):
        self.st(v=(0.3, 0.0), n=50)                                        # drifting: the hold pushes back
        self.assertNotEqual(self.st(v=(0.3, 0.0))[0], 0.0)
        self.assertEqual(self.st(v=(0.5, 0.2), stick=(0.4, 0.0)), (0.0, 0.0))   # stick: pure ANGLE, the hold adds nothing

    def test_release_brakes_then_anchors_where_it_stopped(self):
        self.st(stick=(0.4, 0.0), v=(0.6, 0.0), n=50)
        pitch, _ = self.st(v=(0.6, 0.0))                                   # released, still moving: speed damping only
        self.assertLess(pitch, 0.0); self.assertEqual(self.L.xy_braking(), 1)
        self.st(v=(0.05, 0.0), n=15)                                       # slow for > 0.2 s: anchored here
        self.assertEqual(self.L.xy_braking(), 0); self.assertAlmostEqual(self.L.xy_p(0), 0.05 * 0.02 * 4, delta=0.01)

    def test_the_level_trim_survives_a_reposition(self):
        self.st(v=(0.1, 0.0), n=500)                                       # 10 s of drift: the integral learns a trim
        i0 = self.L.xy_i(0); self.assertGreater(abs(i0), 0.1)
        self.L.xy_reposition(); self.assertAlmostEqual(self.L.xy_p(0), 0.0); self.assertAlmostEqual(self.L.xy_i(0), i0)

    def test_the_climb_learns_the_trim(self):
        self.st(v=(0.0, 0.15), hold=0, n=250)                              # 5 s climbing while drifting right
        self.assertGreater(self.L.xy_i(1), 0.0); self.assertLess(self.st(v=(0.0, 0.0), hold=0)[1], 0.0)   # the trim tilts left against the drift

    def test_the_position_aid_replaces_the_flow_displacement(self):
        self.st(v=(0.2, 0.0), n=50)                                        # flow says 0.2 m forward
        self.st(v=(0.0, 0.0), aid=(-0.1, 0.05))
        self.assertAlmostEqual(self.L.xy_p(0), -0.1, places=4); self.assertAlmostEqual(self.L.xy_p(1), 0.05, places=4)
        self.assertEqual(self.L.xy_aid_used(), 1)
        self.st(v=(0.0, 0.0), aid=(5.0, 0.0)); self.assertEqual(self.L.xy_aid_used(), 0)   # implausible aid (> 3 m): ignored
