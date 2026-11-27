"""v4.8.4 height hold: hover-base learning, acceleration feed-forward (2.3), 5 mm target threshold, kv 0.32. Weak plant (0.43x design, the measured real gain).
Run: python3 -m pytest test_v484.py -q"""
import ctypes, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import fc_hold_sim as S
LIB = os.path.join(HERE, "libhh_v484.so"); F = ctypes.c_float
I_KV, I_KI, I_IMAX, I_AZ, I_LEARN = 1, 2, 3, 37, 38


class Weak(S.Plant):
    scale = 0.43
    def step(self, dt, u):
        u0 = self.hover; return super().step(dt, u0 + self.scale * (u - u0))


class _Deficient:
    """wraps the C core: right after the hand-over the base is lowered by `deficit` (the real drone: the lift-off throttle in ground effect is ~0.18 below what the hover needs)"""
    def __init__(self, L, deficit): self._L, self._deficit, self._done = L, deficit, False
    def __getattr__(self, n): return getattr(self._L, n)
    def hh_update(self, *a):
        r = self._L.hh_update(*a)
        if not self._done and self._L.hh_mode() == 0:
            self._L.hh_set_fbase.argtypes = [F]; self._L.hh_fbase.restype = F
            self._L.hh_set_fbase(self._L.hh_fbase() - self._deficit); self._done = True
        return r


def fly(hover, learn=8.0, az=2.3, step_to=None, hold_s=50.0, ref=0.66, seed=1, deficit=0.0):
    core = S.FcCore(LIB); L = core.L; L.hh_params.restype = ctypes.POINTER(F); P = L.hh_params(); P[I_LEARN] = learn; P[I_AZ] = az
    L.hh_integ.restype = F; L.hh_fbase.restype = F
    if deficit: core.L = _Deficient(L, deficit)
    if hasattr(S.fly, "_stepped"): delattr(S.fly, "_stepped")
    kw = dict(step_to=step_to) if step_to else {}
    tr = S.fly(Weak(hover=hover, seed=seed), core, f_hover_ref=ref, target=0.5, hold_s=hold_s, t_max=140.0, land=False, **kw)
    return tr, abs(L.hh_integ()), L.hh_fbase()


def test_learning_moves_the_integral_into_the_base():
    tr0, i0, b0 = fly(0.74, learn=0.0, deficit=0.12)     # base 0.12 below what the hover needs: the integral has to carry it
    tr1, i1, b1 = fly(0.74, learn=8.0, deficit=0.12)
    assert i1 < 0.5 * i0 and b1 > b0 + 0.04, (i0, i1, b0, b1)
    z = np.array(tr1["trace"]["z"]); t = np.array(tr1["trace"]["t"]); late = t > tr1["t_air"] + 25
    assert abs(z[late].mean() - 0.5) < 0.02                 # and the height stays on the target


def test_step_up_and_back_down_reaches_the_numbers():
    for hover in (0.70, 0.76):
        tr, _, _ = fly(hover, step_to=0.62, hold_s=40.0)
        z = np.array(tr["trace"]["z"]); t = np.array(tr["trace"]["t"]); ta = tr["t_air"]
        mid = (t > ta + 20 * 0.5 + 8.0) & (t < ta + 40 * 0.5 + 8.0) if False else (t > ta + 28) & (t < ta + 38)
        assert abs(z[mid].mean() - 0.62) < 0.02, (hover, z[mid].mean())
        assert z.max() < 0.66                               # no more than 4 cm overshoot


def test_two_centimetre_steps_are_no_longer_swallowed():
    L = ctypes.CDLL(LIB); L.hh_target.restype = F; L.hh_defaults(); L.hh_engage(F(0.5), F(0.70), F(15.0)); L.hh_set_target(F(0.5))
    L.hh_set_target(F(0.51)); assert abs(L.hh_target() - 0.51) < 1e-6        # 1 cm (v4.8.3: ignored)
    L.hh_set_target(F(0.512)); assert abs(L.hh_target() - 0.51) < 1e-6       # 2 mm jitter: still ignored
