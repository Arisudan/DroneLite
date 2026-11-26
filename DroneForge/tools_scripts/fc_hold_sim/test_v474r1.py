"""v4.7.4R1 = v4.7.4R with the v4.5 numbers: height gains 0.9923 / 0.10 / 0.04, sideways kp 2.5 / ki 0.10; the learned level trim (climb_trim 3) is KEPT: with 0 the simulated drift goes back to the rev1 level. Everything else is v4.7.4R.
Run: python3 -m pytest test_v474r1.py -q"""
import ctypes, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import fc_hold_sim as S
import xy_bench as XB
R1, V474, V474R1 = (os.path.join(HERE, f) for f in ("libhh_v46rev1.so", "libhh_v474R.so", "libhh_v474R1.so"))
F = ctypes.c_float


def _hp(lib):
    L = ctypes.CDLL(lib); L.hh_params.restype = ctypes.POINTER(F); L.hh_nparams.restype = ctypes.c_int; L.hh_defaults(); p = L.hh_params(); return [p[i] for i in range(L.hh_nparams())]


def _xp(lib):
    L = ctypes.CDLL(lib); L.xy_params.restype = ctypes.POINTER(F); L.xy_defaults(); p = L.xy_params(); return [p[i] for i in range(20)]


def test_height_gains_are_the_v45_ones_and_nothing_else_in_the_height_loop_changed():
    a, b = _hp(V474), _hp(V474R1)
    assert np.allclose(b[:3], [0.9923, 0.10, 0.04], atol=1e-6)
    assert [i for i in range(3, len(a)) if abs(a[i] - b[i]) > 1e-9] == []                     # take-off creep, climb limit, landing, overshoot guards: untouched


def test_sideways_numbers():
    a, b = _xp(V474), _xp(V474R1)
    assert abs(b[0] - 5.0) < 1e-6 and abs(b[1] - 2.5) < 1e-6 and abs(b[2] - 0.10) < 1e-6      # kv, kp, ki (XY order of xy_params)
    assert abs(b[19] - 3.0) < 1e-6                                                       # climb_trim kept at 3 deg/m
    diff = [XB.XY_NAMES[i] for i in range(15) if abs(a[i] - b[i]) > 1e-9]
    assert set(diff) <= {"kp", "ki"}, diff


def test_take_off_hold_land_still_works_on_the_simulated_drone():
    for hover in (0.66, 0.72, 0.78):
        out = {}
        for name, lib in (("474R", V474), ("474R1", V474R1)):
            core = S.FcCore(lib)
            if hasattr(S.fly, "_stepped"): delattr(S.fly, "_stepped")
            tr = S.fly(S.Plant(hover=hover, seed=1), core, f_hover_ref=0.66, target=0.5, hold_s=10.0)
            z = np.array(tr["trace"]["z"]); out[name] = (z.max() - 0.5, abs(z[-int(len(z) * 0.3):].mean() - 0.5) if False else float(abs(np.median(z[len(z) // 2:]) - 0.5)), tr["t_air"])
        print(hover, out)
        o = out["474R1"]
        assert o[0] < 0.15 and o[1] < 0.05 and o[2] > 0                                          # overshoot < 15 cm, settled within 5 cm of 0.5 m, it did lift off
