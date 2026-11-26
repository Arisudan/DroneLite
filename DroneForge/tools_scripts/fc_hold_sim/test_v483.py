"""v4.8.3: a saturated motor must stop the hold from winding up (integral frozen, throttle not raised) while the drone is below the target; unsaturated behaviour unchanged.
Run: python3 -m pytest test_v483.py -q"""
import ctypes, os, sys
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
LIB = os.path.join(HERE, "libhh_v483.so"); F = ctypes.c_float


def _hold(sat):
    L = ctypes.CDLL(LIB); L.hh_update.restype = F; L.hh_update.argtypes = [F, F, F, F]; L.hh_integ.restype = F; L.hh_f_out.restype = F
    L.hh_engage.argtypes = [F, F, F]; L.hh_set_target.argtypes = [F]; L.hh_set_motor_sat.argtypes = [ctypes.c_int]
    L.hh_defaults(); L.hh_engage(F(0.5), F(0.70), F(15.0)); L.hh_set_target(F(0.5))
    f0 = L.hh_update(F(0.02), F(0.5), F(0.0), F(15.0)); out = []
    for i in range(300):                                   # 6 s, the drone 10 cm BELOW the target and not rising
        L.hh_set_motor_sat(1 if sat else 0); out.append(L.hh_update(F(0.02), F(0.40), F(0.0), F(15.0)))
    return f0, out, L.hh_integ()


def test_saturated_motor_freezes_the_integral_and_the_throttle():
    f0, out, integ = _hold(True)
    assert max(out) <= out[0] + 1e-6, (out[0], max(out))              # the throttle never grows beyond its first value while saturated
    assert integ < 0.05                                               # the integral did not wind up (unsaturated it would reach ~0.06 * 0.1 * 6 = 0.036 + the proportional part)


def test_without_saturation_the_throttle_keeps_rising_as_before():
    f0, out, integ = _hold(False)
    assert out[-1] > out[0] + 0.01                                    # the integral keeps adding throttle
