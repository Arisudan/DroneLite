"""Test bench for the FC sideways loop (firmware flight/xy_hold_core.c compiled on the PC) on a simulated horizontal plant:
ANGLE-mode tilt (tau 0.08 s) -> acceleration g*tan(tilt) + a constant push (a level error) - drag, EKF body speed with 0.08 s delay and noise."""
import ctypes, math, os, random
HERE = os.path.dirname(os.path.abspath(__file__)); G = 9.81
L = ctypes.CDLL(os.path.join(HERE, "libhh.so"))
L.xy_update.argtypes = [ctypes.c_float] * 4 + [ctypes.c_int, ctypes.POINTER(ctypes.c_float)]
L.xy_sign.restype = ctypes.c_float; L.xy_p.restype = ctypes.c_float; L.xy_params.restype = ctypes.POINTER(ctypes.c_float)
XY_NAMES = ["kv", "kp", "ki", "max_deg", "min_z", "test_z", "test_deg", "test_s", "test_min_dv", "calm_speed", "calm_s", "runaway_speed", "runaway_s", "runaway_pos", "skip_test"]

def run(push=(0.14, 0.0), sign_meas=(1.0, 1.0), resp=(1.0, 1.0), t_end=40.0, delay=0.08, noise=0.03, seed=1, z=1.0, over=None, loop=True, v0=(0.0, 0.0)):
    """sign_meas: -1 = that measured axis is inverted relative to the tilt command (wrong sign); resp: 0 = that axis does not respond to tilt."""
    rng = random.Random(seed); L.xy_defaults()
    p = L.xy_params()
    for k, v in (over or {}).items(): p[XY_NAMES.index(k)] = v
    dt = 0.002; tau = 0.08; tilt = [0.0, 0.0]; vel = [v0[0], v0[1]]; pos = [0.0, 0.0]; hist = []; cmd = [0.0, 0.0]; out = (ctypes.c_float * 2)()
    t = 0.0; n = 0; maxd = 0.0; maxv = 0.0; trace = []
    while t < t_end:
        if n % 10 == 0:                                    # 50 Hz FC step
            tm = t - delay
            vm = vel
            for (th, hv) in reversed(hist):
                if th <= tm: vm = hv; break
            fwd = sign_meas[0] * vm[0] + rng.gauss(0, noise); right = sign_meas[1] * vm[1] + rng.gauss(0, noise)
            if loop:
                L.xy_update(0.02, fwd, right, z, 1, out); cmd = [out[1], out[0]]     # (pitch, roll) deg: out = (roll, pitch)
            else: cmd = [0.0, 0.0]
        hist.append((t, (vel[0], vel[1])))
        if len(hist) > 400: hist.pop(0)
        for a in (0, 1):
            tilt[a] += (resp[a] * cmd[a] - tilt[a]) * dt / tau
            acc = G * math.tan(math.radians(tilt[a])) + push[a] - 0.15 * vel[a]
            vel[a] += acc * dt; pos[a] += vel[a] * dt
        t += dt; n += 1
        d = math.hypot(*pos); maxd = max(maxd, d); maxv = max(maxv, math.hypot(*vel))
        if n % 500 == 0: trace.append((round(t, 1), round(pos[0], 2), round(pos[1], 2), round(vel[0], 2), round(vel[1], 2), cmd[0], cmd[1]))
    return dict(max_dist=maxd, end_dist=math.hypot(*pos), max_speed=maxv, end_speed=math.hypot(*vel), disabled=bool(L.xy_disabled()), phase=L.xy_phase(),
                sign=(L.xy_sign(0), L.xy_sign(1)), axis_off=(bool(L.xy_axis_off(0)), bool(L.xy_axis_off(1))), trace=trace)

if __name__ == "__main__":
    for name, kw in (("no loop, push 0.14", dict(loop=False)), ("loop, push 0.14", {}), ("loop, push both 0.10/-0.10", dict(push=(0.10, -0.10))),
                     ("loop, roll sign inverted", dict(sign_meas=(1.0, -1.0), push=(0.10, 0.10))), ("loop, both inverted", dict(sign_meas=(-1.0, -1.0), push=(0.10, 0.10))),
                     ("loop, roll axis dead", dict(resp=(1.0, 0.0), push=(0.10, 0.10)))):
        r = run(**kw); print(f"{name:30} max dist {r['max_dist']:.2f}  end dist {r['end_dist']:.2f}  max speed {r['max_speed']:.2f}  disabled {r['disabled']}  phase {r['phase']}  sign {r['sign']}  axis_off {r['axis_off']}")
