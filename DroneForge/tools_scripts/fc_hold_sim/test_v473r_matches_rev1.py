"""v4.7.3R must fly like v4.6-rev1 (the last build the owner trusts) and keep only the v4.7.2 VO position aid.
Compares the two compiled hold laws (libhh_v473R.so, libhh_v46rev1.so) on identical simulated flights. Run: python3 -m pytest test_v473r_matches_rev1.py -q"""
import ctypes, os, random, sys
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import fc_hold_sim as S
import xy_bench as XB
REV1, NEW = os.path.join(HERE, "libhh_v46rev1.so"), os.path.join(HERE, "libhh_v473R.so")
F = ctypes.c_float


def test_every_rev1_parameter_is_unchanged():
    a, b = S.FcCore(REV1), S.FcCore(NEW)
    assert [(S.NAMES[i], a.p[i], b.p[i]) for i in range(a.L.hh_nparams()) if abs(a.p[i] - b.p[i]) > 1e-9] == []           # height / take-off / landing
    assert b.L.hh_nparams() == a.L.hh_nparams() + 2 and b.get("takeoff_closed") == 0.0 and b.p[b.L.hh_nparams() - 1] == 0.0    # the two new switches are OFF
    for L in (a.L, b.L): L.xy_params.restype = ctypes.POINTER(ctypes.c_float); L.xy_defaults()
    xa, xb = a.L.xy_params(), b.L.xy_params()
    assert [XB.XY_NAMES[i] for i in range(15) if abs(xa[i] - xb[i]) > 1e-9] == []                                           # sideways gains and limits
    assert abs(xb[15] - 2.0) < 1e-6 and abs(xb[19]) < 1e-9        # stick_db 2.0 (stick-through can never trigger), climb_trim 0


def test_height_flights_are_identical_to_rev1_to_float_rounding():
    worst = 0.0
    for hover in (0.66, 0.78):
        for seed in (1, 2):
            for kw in (dict(target=0.5), dict(target=0.6, hold_s=14.0, step_to=0.8), dict(target=0.5, hold_s=8.0, cap_land_s=1.5)):
                tr = []
                for lib in (REV1, NEW):
                    core = S.FcCore(lib)
                    if hasattr(S.fly, "_stepped"): delattr(S.fly, "_stepped")
                    tr.append(S.fly(S.Plant(hover=hover, seed=seed), core, f_hover_ref=0.66, **kw))
                a, b = tr[0]["trace"], tr[1]["trace"]
                assert len(a["thr"]) == len(b["thr"]) and tr[0]["t_air"] == tr[1]["t_air"]                                      # same take-off time, same end
                worst = max(worst, max(abs(x - y) for x, y in zip(a["thr"], b["thr"])), max(abs(x - y) for x, y in zip(a["z"], b["z"])))
    assert worst < 1e-5, worst


def test_sideways_loop_ignores_sticks_and_matches_rev1():
    a, b = S.FcCore(REV1), S.FcCore(NEW); oa, ob = (F * 2)(), (F * 2)(); rng = random.Random(5); worst = 0.0
    a.L.xy_update.argtypes = [F] * 4 + [ctypes.c_int, ctypes.POINTER(F)]
    b.L.xy_update_st.argtypes = [F] * 4 + [ctypes.c_int, F, F, ctypes.c_int, F, F, ctypes.POINTER(F)]
    for trial in range(6):
        a.L.xy_defaults(); b.L.xy_defaults()
        for k in range(2000):
            fw, rt, z, hold = rng.gauss(0, 0.3), rng.gauss(0, 0.3), 0.55 if k > 60 else 0.03 + k * 0.008, k > 700
            sp, sr = (rng.uniform(-1, 1), rng.uniform(-1, 1)) if trial % 2 else (0.0, 0.0)
            a.L.xy_update(F(0.02), F(fw), F(rt), F(z), int(hold), oa); b.L.xy_update_st(F(0.02), F(fw), F(rt), F(z), int(hold), sp, sr, 0, 0.0, 0.0, ob)
            worst = max(worst, abs(oa[0] - ob[0]), abs(oa[1] - ob[1]))
            if k in (1000, 1500): a.L.xy_reposition(); b.L.xy_reposition()                                                 # hand-over / return from ALT HOLD
    assert worst < 1e-6, worst


def test_the_vo_aid_is_kept():
    b = S.FcCore(NEW); ob = (F * 2)(); b.L.xy_defaults(); b.L.xy_update_st.argtypes = [F] * 4 + [ctypes.c_int, F, F, ctypes.c_int, F, F, ctypes.POINTER(F)]
    for _ in range(5): b.L.xy_update_st(F(0.02), F(0.0), F(0.0), F(0.55), 1, 0.0, 0.0, 1, 0.20, 0.0, ob)
    assert b.L.xy_aid_used() == 1 and abs(abs(ob[1]) - 2.4 * 0.20) < 0.02          # 0.20 m of aid x the rev1 kp 2.4 deg/m = 0.48 deg
