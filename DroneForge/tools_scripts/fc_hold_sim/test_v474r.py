"""v4.7.4R = v4.7.3R (rev1 feel) + the level-trim learning in the climb (POS HOLD). Compares the compiled hold laws (libhh_v474R.so, libhh_v473R.so, libhh_v46rev1.so).
Run: python3 -m pytest test_v474r.py -q"""
import ctypes, os, random, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import fc_hold_sim as S
import xy_bench as XB
REV1, V473, V474 = (os.path.join(HERE, f) for f in ("libhh_v46rev1.so", "libhh_v473R.so", "libhh_v474R.so"))
F = ctypes.c_float


class _NoBrake:                         # the firmware repositions at the hand-over (v4.7.3R / v4.7.4R): the bench must not brake first
    def __init__(self, L): self.L = L
    def __getattr__(self, n):
        if n == "xy_brake": raise AttributeError(n)
        return getattr(self.L, n)


def _bench(lib, seeds=12, **kw):
    L = ctypes.CDLL(lib); L.xy_update.argtypes = [F] * 4 + [ctypes.c_int, ctypes.POINTER(F)]; L.xy_params.restype = ctypes.POINTER(F)
    if hasattr(L, "xy_update_st"): L.xy_update_st.argtypes = [F] * 4 + [ctypes.c_int, F, F, ctypes.c_int, F, F, ctypes.POINTER(F)]
    return np.array([XB.run(_NoBrake(L), s, **kw)["max_d"] for s in range(seeds)])


def test_height_take_off_and_landing_are_still_identical_to_rev1():
    worst = 0.0
    for hover in (0.66, 0.78):
        for seed in (1, 2):
            for kw in (dict(target=0.5), dict(target=0.6, hold_s=14.0, step_to=0.8), dict(target=0.5, hold_s=8.0, cap_land_s=1.5)):
                tr = []
                for lib in (REV1, V474):
                    core = S.FcCore(lib)
                    if hasattr(S.fly, "_stepped"): delattr(S.fly, "_stepped")
                    tr.append(S.fly(S.Plant(hover=hover, seed=seed), core, f_hover_ref=0.66, **kw))
                a, b = tr[0]["trace"], tr[1]["trace"]; assert len(a["thr"]) == len(b["thr"]) and tr[0]["t_air"] == tr[1]["t_air"]
                worst = max(worst, max(abs(x - y) for x, y in zip(a["thr"], b["thr"])), max(abs(x - y) for x, y in zip(a["z"], b["z"])))
    assert worst < 1e-5, worst


def test_sideways_gains_and_limits_are_still_rev1_only_the_trim_is_new():
    a, b = S.FcCore(REV1), S.FcCore(V474)
    for L in (a.L, b.L): L.xy_params.restype = ctypes.POINTER(F); L.xy_defaults()
    xa, xb = a.L.xy_params(), b.L.xy_params()
    assert [XB.XY_NAMES[i] for i in range(15) if abs(xa[i] - xb[i]) > 1e-9] == []           # kv 5 / kp 2.4 / ki 0.15 / 4 deg / runaway guard: the rev1 numbers
    assert abs(xb[15] - 2.0) < 1e-6 and abs(xb[19] - 3.0) < 1e-6                              # stick-through off (stick_db 2.0), climb_trim 3 deg/m (new)


def test_the_trim_cuts_the_position_wander():
    rev1, v473, v474 = _bench(REV1), _bench(V473), _bench(V474)
    assert abs(np.median(v473) - np.median(rev1)) < 1e-6                                       # v4.7.3R really was rev1
    assert np.median(v474) < 0.55 * np.median(rev1) and np.median(v474) < 0.25                # flow only: 0.37 m -> 0.17 m
    aid = _bench(V474, aid=True)
    assert np.median(aid) < 0.09 and aid.max() < 0.12                                          # with the VO aid: 0.43 m -> 0.05 m


def test_the_learned_trim_survives_the_hand_over_and_a_new_flight_clears_it():
    b = S.FcCore(V474); L = b.L; L.xy_params.restype = ctypes.POINTER(F); L.xy_i.restype = F; out = (F * 2)()
    L.xy_update.argtypes = [F] * 4 + [ctypes.c_int, ctypes.POINTER(F)]; L.xy_defaults()
    for _ in range(300): L.xy_update(F(0.02), F(0.15), F(0.0), F(0.3), 0, out)                # the climb, drifting forward at 0.15 m/s: the trim is learned
    learned = L.xy_i(0); assert abs(learned) > 1.0
    L.xy_reposition(); assert abs(L.xy_i(0) - learned) < 1e-6                                   # the hand-over (xyReposition) keeps it
    L.xy_defaults(); assert abs(L.xy_i(0)) < 1e-9                                               # a new flight (xyStart) clears it


def test_sticks_still_never_change_anything_and_the_aid_still_works():
    b = S.FcCore(V474); ob = (F * 2)()                                                           # one library = one static state: run the two cases one after the other
    b.L.xy_update_st.argtypes = [F] * 4 + [ctypes.c_int, F, F, ctypes.c_int, F, F, ctypes.POINTER(F)]

    def run(deflected: bool):
        rng = random.Random(5); rs = random.Random(11); b.L.xy_defaults(); out = []        # the sticks have their own generator: the other inputs stay identical
        for k in range(3000):
            fw, rt, z, hold = rng.gauss(0, 0.3), rng.gauss(0, 0.3), 0.55 if k > 60 else 0.03 + k * 0.008, k > 700
            sp, sr = (rs.uniform(-1, 1), rs.uniform(-1, 1)) if deflected else (0.0, 0.0)
            b.L.xy_update_st(F(0.02), F(fw), F(rt), F(z), int(hold), sp, sr, 0, 0.0, 0.0, ob); out.append((ob[0], ob[1]))
        return np.array(out)
    centred, deflected = run(False), run(True)
    assert np.abs(centred - deflected).max() < 1e-6 and np.abs(centred).max() > 0.1             # deflected sticks: no effect (stick-through stays off), and the loop did act
    b.L.xy_defaults(); [b.L.xy_update_st(F(0.02), F(0.0), F(0.0), F(0.55), 1, 0.0, 0.0, 1, 0.20, 0.0, ob) for _ in range(5)]
    assert b.L.xy_aid_used() == 1 and abs(abs(ob[1]) - 2.4 * 0.20) < 0.02                      # 0.20 m of aid x kp 2.4 = 0.48 deg
