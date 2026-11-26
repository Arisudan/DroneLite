"""v4.8 (built, not flashed) = v4.7.4R1 + the VO position aid also acts in the take-off climb (xyParams.climb_aid) + the take-off creep starts at 0.92 (was 0.82) of the hover
reference. (The shorter state frame is tested in software/pc/nimbus_pc/tests/test_state_frame_v48.py.) Run: python3 -m pytest test_v48.py -q"""
import ctypes, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import fc_hold_sim as S
import xy_bench as XB
BASE, V48 = (os.path.join(HERE, f) for f in ("libhh_v474R1.so", "libhh_v48.so"))
F = ctypes.c_float


def _lib(path):
    L = ctypes.CDLL(path); L.xy_params.restype = ctypes.POINTER(F)
    L.xy_update.argtypes = [F] * 4 + [ctypes.c_int, ctypes.POINTER(F)]
    L.xy_update_st.argtypes = [F] * 4 + [ctypes.c_int, F, F, ctypes.c_int, F, F, ctypes.POINTER(F)]
    return L


def _climb_out(path, aid, z=0.3, climb_aid=None):
    L = _lib(path); L.xy_defaults(); out = (F * 2)()
    if climb_aid is not None: L.xy_params()[20] = climb_aid
    for _ in range(3): L.xy_update_st(0.02, 0.0, 0.0, z, 0, 0.0, 0.0, aid, 0.20, 0.0, out)
    return out[0], out[1]


def test_the_aid_acts_in_the_climb_only_in_v48():
    assert abs(_climb_out(BASE, 1)[1]) < 1e-6                                   # v4.7.4R1 ignores it until HOLD
    assert abs(abs(_climb_out(V48, 1)[1]) - 2.5 * 0.20) < 0.02                  # kp 2.5 x 0.20 m
    assert abs(_climb_out(V48, 1, climb_aid=0.0)[1]) < 1e-6                     # switch off = v4.7.4R1
    assert abs(_climb_out(V48, 0)[1]) < 1e-6                                    # no valid aid: nothing
    assert abs(_climb_out(V48, 1, z=0.03)[1]) < 1e-6                            # on the floor: nothing


def test_ground_key_frame_reduces_climb_drift_and_leaves_hover_anchor_unchanged():
    class NoBrake:
        def __init__(self, L): self.L = L
        def __getattr__(self, n):
            if n == "xy_brake": raise AttributeError(n)
            return getattr(self.L, n)

    def runs(path, **kw): return [XB.run(NoBrake(_lib(path)), s, **kw) for s in range(12)]
    old, new = runs(BASE, ground_aid=True), runs(V48, ground_aid=True)
    assert np.median([r["max_d"] for r in new]) < 0.75 * np.median([r["max_d"] for r in old])
    a, b = runs(BASE, aid=True), runs(V48, aid=True)
    assert max(abs(x["max_d"] - y["max_d"]) for x, y in zip(a, b)) < 1e-6


def test_only_the_take_off_start_changed_in_the_height_loop():
    def params(path):
        L = ctypes.CDLL(path); L.hh_params.restype = ctypes.POINTER(F); L.hh_nparams.restype = ctypes.c_int; L.hh_defaults(); p = L.hh_params()
        return [p[i] for i in range(L.hh_nparams())]
    a, b = params(BASE), params(V48)
    i = S.NAMES.index("takeoff_start_frac")
    assert [k for k in range(len(a)) if abs(a[k] - b[k]) > 1e-6] == [i] and abs(b[i] - 0.92) < 1e-6


def test_lift_off_is_sooner_and_the_climb_no_wilder():
    for hover in (0.66, 0.72, 0.78, 0.82):
        res = {}
        for name, lib in (("base", BASE), ("v48", V48)):
            runs = []
            for seed in (1, 2, 3):
                for ge in (0.15, 0.25, 0.45):
                    core = S.FcCore(lib)
                    if hasattr(S.fly, "_stepped"): delattr(S.fly, "_stepped")
                    runs.append(S.fly(S.Plant(hover=hover, ge=ge, seed=seed, vz_bias=0.2), core, f_hover_ref=0.66, target=0.5, hold_s=10.0, t_max=45.0))
            assert not any(r["fail"] or r["crashed"] or r["t_air"] is None for r in runs)
            res[name] = (np.median([r["t_air"] for r in runs]), max(r["zmax"] for r in runs), max(r["vzmax"] for r in runs))
        assert res["v48"][0] < res["base"][0] - 1.2, (hover, res)
        assert res["v48"][1] < res["base"][1] + 0.08 and res["v48"][2] < res["base"][2] + 0.20, (hover, res)
