"""Realistic bench for the FC sideways hold (firmware xy_hold_core.c compiled on the PC, libhh.so): the drone's measured imperfections
(2026-11-21: MTF-02P flow 25 Hz, 0.081 m/s noise (kf_calib.json), 50 ms old, EKF low-pass ~0.1 s, +-3 % scale error; level reference 1.0 deg off (logs: 0.75-1 deg,
drift ~0.17 m/s^2); ANGLE-mode tilt lag 0.08 s; linear drag 0.15 /s). Reports the TRUE drift over N seeded 60 s hovers.
   python3 xy_bench.py [lib.so] [key=value ...]   (xy parameter overrides, e.g. ki=0.6)"""
import ctypes, math, random, sys, os
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); G = 9.81
XY_NAMES = ["kv", "kp", "ki", "max_deg", "min_z", "test_z", "test_deg", "test_s", "test_min_dv", "calm_speed", "calm_s", "runaway_speed", "runaway_s", "runaway_pos", "skip_test"]

def run(L, seed, t_end=67.0, level_deg=(1.0, 0.0), over=None, sigma=0.081, scale_sd=0.03, flow_hz=25.0, delay=0.05, ekf_tau=0.10, aid=False, aid_hz=10.0, aid_sd=0.02, aid_delay=0.15, climb_s=7.0):
    rng = random.Random(seed); L.xy_defaults(); p = L.xy_params()
    for k, v in (over or {}).items(): p[XY_NAMES.index(k)] = v
    L.xy_start() if hasattr(L, "xy_start") else None
    scale = [1.0 + rng.gauss(0, scale_sd), 1.0 + rng.gauss(0, scale_sd)]
    push = [G * math.tan(math.radians(level_deg[1])), G * math.tan(math.radians(level_deg[0]))]   # (forward, right) acceleration from the level error
    dt = 0.002; tilt = [0.0, 0.0]; vel = [0.0, 0.0]; pos = [0.0, 0.0]; hist = []; cmd = [0.0, 0.0]; out = (ctypes.c_float * 2)()
    meas = [0.0, 0.0]; est = [0.0, 0.0]; t = 0.0; n = 0; t_flow = 0.0; trace = []; phist = []; t_aid = -1.0; aidv = None; held = False; anchor = [0.0, 0.0]; braking = False
    while t < t_end:
        hist.append((t, vel[0], vel[1])); phist.append((t, pos[0], pos[1]))
        if len(hist) > 200: hist.pop(0)
        if len(phist) > 300: phist.pop(0)
        if aid and held and not braking and t - t_aid >= 1.0 / aid_hz:                      # PC visual odometry: displacement from the hold point (no yaw here: body = world)
            t_aid = t; o = min(phist, key=lambda h: abs(h[0] - (t - aid_delay))); aidv = (o[1] - anchor[0] + rng.gauss(0, aid_sd), o[2] - anchor[1] + rng.gauss(0, aid_sd))
        if t - t_flow >= 1.0 / flow_hz:
            t_flow = t; old = min(hist, key=lambda h: abs(h[0] - (t - delay)))
            meas = [scale[0] * old[1] + rng.gauss(0, sigma), scale[1] * old[2] + rng.gauss(0, sigma)]
        if n % 10 == 0:                                                       # FC loop 50 Hz
            for a in (0, 1): est[a] += (meas[a] - est[a]) * min(1.0, 0.02 / ekf_tau)
            hold = t >= climb_s
            if hold and not held:                                                   # the hand-over
                if hasattr(L, "xy_brake"): L.xy_brake(); braking = True                # v4.7: brake first, the hold point is where it stopped
                else: L.xy_reposition(); braking = False
                held = True; anchor = list(pos); phist = [(t, 0.0, 0.0)]; aidv = None; t_hold = t
            if held and braking and not L.xy_braking(): braking = False; anchor = list(pos); phist = [(t, 0.0, 0.0)]; aidv = None   # stopped: the GCS anchors its VO key frame by the same rule
            rel = (pos[0] - anchor[0], pos[1] - anchor[1]) if held else (0.0, 0.0)
            if aid and held and not braking: L.xy_update_st(0.02, est[0], est[1], 0.8, 1, 0.0, 0.0, aidv is not None, aidv[0] if aidv else 0.0, aidv[1] if aidv else 0.0, out)
            else: L.xy_update(0.02, est[0], est[1], 0.8, 1 if hold else 0, out)
            cmd = [out[1], out[0]]     # (pitch, roll) deg
        for a in (0, 1):
            tilt[a] += (cmd[a] - tilt[a]) * dt / 0.08
            vel[a] += (G * math.tan(math.radians(tilt[a])) + push[a] - 0.15 * vel[a]) * dt; pos[a] += vel[a] * dt
        t += dt; n += 1
        if n % 500 == 0 and held: trace.append(math.hypot(pos[0] - anchor[0], pos[1] - anchor[1]))
    return dict(max_d=max(trace), end_d=trace[-1], last20=np.mean(trace[-20:]), trace=trace)

def bench(lib, over=None, seeds=20, **kw):
    L = ctypes.CDLL(lib); L.xy_update.argtypes = [ctypes.c_float] * 4 + [ctypes.c_int, ctypes.POINTER(ctypes.c_float)]; L.xy_params.restype = ctypes.POINTER(ctypes.c_float)
    if hasattr(L, "xy_update_st"): L.xy_update_st.argtypes = [ctypes.c_float] * 4 + [ctypes.c_int, ctypes.c_float, ctypes.c_float, ctypes.c_int, ctypes.c_float, ctypes.c_float, ctypes.POINTER(ctypes.c_float)]
    R = [run(L, s, over=over, **kw) for s in range(seeds)]
    m = np.array([r["max_d"] for r in R]); e = np.array([r["end_d"] for r in R])
    return f"max drift median {np.median(m)*1000:.0f} mm, p90 {np.percentile(m,90)*1000:.0f} mm, worst {m.max()*1000:.0f} mm | at 60 s median {np.median(e)*1000:.0f} mm"

if __name__ == "__main__":
    lib = sys.argv[1] if len(sys.argv) > 1 and sys.argv[1].endswith(".so") else os.path.join(HERE, "libhh.so")
    over = dict((a.split("=")[0], float(a.split("=")[1])) for a in sys.argv[1:] if "=" in a)
    print("level error only     :", bench(lib, over, sigma=0.0, scale_sd=0.0))
    print("flow noise+scale only:", bench(lib, over, level_deg=(0.0, 0.0)))
    print("all (realistic)      :", bench(lib, over))
    if "v47" in lib or "v4.7" in lib: print("all + VO aid (v4.7)  :", bench(lib, over, aid=True))
