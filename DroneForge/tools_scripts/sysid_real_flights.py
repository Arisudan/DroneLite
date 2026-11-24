import csv, glob, math, sys, numpy as np
from scipy.optimize import least_squares
G = 9.81; IDLE = 0.05
def load(F):
    rows = []
    for x in csv.DictReader(open(F)):
        try: t = float(x["t"]); th = float(x["sent_thr"]); rg = float(x["range_m"])
        except: continue
        if x["phase"] != "flying": continue
        rows.append((t, th, rg))
    return np.array(rows)
def windows(a, zmin=0.5, minlen=1.2):
    out = []; i = 0; n = len(a)
    while i < n:
        if a[i, 2] > zmin:
            j = i
            while j + 1 < n and a[j + 1, 2] > zmin and a[j + 1, 0] - a[j, 0] < 0.3: j += 1
            if a[j, 0] - a[i, 0] >= minlen: out.append((i, j))
            i = j + 1
        else: i += 1
    return out
def simulate(a, i, j, p, z0, v0, pre=0.6):
    h, tau, td, drag = p
    t = a[i:j + 1, 0]; thr = a[i:j + 1, 1]
    # throttle history before the window (pre) is the first value held
    dt = 0.01; T = np.arange(t[0], t[-1] + 1e-9, dt)
    cmd = np.interp(T - td, t, thr, left=thr[0])
    te = np.empty_like(T); te0 = thr[0]; x = te0
    for k in range(len(T)):
        x += (cmd[k] - x) * dt / tau; te[k] = x
    z = z0; v = v0; Z = np.empty_like(T)
    for k in range(len(T)):
        u = max(te[k] - IDLE, 0.0) / max(h - IDLE, 1e-6)
        az = G * (u * u - 1.0) - drag * v
        v += az * dt; z += v * dt; Z[k] = z
    return np.interp(t, T, Z)
def fit(files, fixed=None):
    data = []
    for F in files:
        a = load(F)
        for (i, j) in windows(a): data.append((a, i, j))
    if not data: return None
    def resid(q):
        h, tau, td, drag = q[:4]; r = []
        for k, (a, i, j) in enumerate(data):
            z0 = q[4 + 2 * k]; v0 = q[5 + 2 * k]
            r.append(simulate(a, i, j, (h, tau, td, drag), z0, v0) - a[i:j + 1, 2])
        return np.concatenate(r)
    q0 = [0.70, 0.06, 0.25, 0.15]; lo = [0.5, 0.0599, 0.0, 0.1499]; hi = [0.95, 0.0601, 1.0, 0.1501]
    for (a, i, j) in data: q0 += [a[i, 2], 0.0]; lo += [0.0, -6]; hi += [6, 6]
    res = least_squares(resid, q0, bounds=(lo, hi), x_scale=[0.05, 0.05, 0.05, 0.1] + [0.2, 1.0] * len(data))
    n = len(res.fun); rms = math.sqrt(np.mean(res.fun ** 2))
    return res.x[:4], rms, len(data), n
if __name__ == "__main__":
    base = "/home/htic/Music/DroneForge/archive/flight_recordings_2026-11-12_to_2026-11-14/"
    sets = {"ALL 11-12 + 11-13": sorted(glob.glob(base + "2026-11-1[23]_*_real-fc.csv"))}
    for F in sorted(glob.glob(base + "2026-11-1[23]_*_real-fc.csv")):
        r = fit([F])
        if r: sets[F.split("/")[-1][:22]] = [F]
    for name, files in sets.items():
        r = fit(files)
        if r is None: continue
        (h, tau, td, drag), rms, nw, n = r
        print(f"{name:24} windows {nw:2d} pts {n:4d}  hover {h:.3f}  tau {tau:.3f}  delay {td:.3f}  drag {drag:.2f}  rms {rms:.3f} m", flush=True)
