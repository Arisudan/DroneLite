"""Replay the firmware's height-hold core (libhh.so, compiled from the firmware source) on the real range and battery data of an FC blackbox log.
Open loop: the sensors come from the real flight, so the output shows what the hold law commands at the real hand-over. usage: replay_real.py LOG.BFL [target_m] [hover_ref] [ekf_lead_s]  (the EKF height leads the range sensor by about this much: z = range + lead * vz)"""
import sys, os, ctypes
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "software", "pc")); sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from nimbus_gcs import bfl
from fc_hold_sim import FcCore
log = sys.argv[1]; target = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5; ref = float(sys.argv[3]) if len(sys.argv) > 3 else 0.66; lead = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
lg = bfl.Log(log); lg.decode(); c = lg.column
t = [x / 1e6 for x in c("time")]; t0 = t[0]; lo, hi = lg.motor_output
M = [c(f"motor[{i}]") for i in range(4)]; mean = [sum((M[k][i] - lo) / (hi - lo) for k in range(4)) / 4 for i in range(len(t))]
rng = [x / 100.0 for x in c("surfaceRaw")]; vbat = [x / 100.0 for x in c("vbatLatest")]
i_ramp = next(i for i in range(len(t)) if mean[i] > 0.12)          # the FC's take-off ramp leaves idle (0.05)
# 50 Hz samples from the log
idx = []; tn = t[i_ramp]
for i in range(i_ramp, len(t)):
    if t[i] >= tn: idx.append(i); tn += 0.02
core = FcCore(os.environ.get("HH_LIB") or os.path.join(os.path.dirname(os.path.abspath(__file__)), "libhh.so")); L = core.L
for kv in os.environ.get("HH_SET", "").split():
    k, v = kv.split("="); core.set(**{k: float(v)})
ground = sorted(rng[max(0, i_ramp - 4000):i_ramp])[len(range(max(0, i_ramp - 4000), i_ramp)) // 2]
vb0 = sum(vbat[i_ramp:i_ramp + 200]) / 200.0
L.hh_takeoff(ctypes.c_float(ref), ctypes.c_float(target), ctypes.c_float(vb0))
zs = []; prev_mode = None; rows = []
for k, i in enumerate(idx):
    z = rng[i] - ground; zs.append(z)
    zf = sum(zs[-5:]) / len(zs[-5:]); zo = sum(zs[-30:-25]) / 5 if len(zs) > 30 else zf
    vz = (zf - zo) / 0.5 if len(zs) > 30 else 0.0
    vb = vbat[i] if vbat[i] > 6 else vb0
    f = L.hh_update(ctypes.c_float(0.02), ctypes.c_float(zf + lead * vz), ctypes.c_float(vz), ctypes.c_float(vb))
    mode = L.hh_mode()
    rows.append((t[i] - t0, zf, vz, vb, f, mode, L.hh_integ(), L.hh_base(), L.hh_zref(), mean[i]))
    if prev_mode is not None and mode != prev_mode: print(f"--- mode {prev_mode} -> {mode} at t={t[i]-t0:.2f} s, z={zf:.2f} m, vz={vz:.2f}")
    prev_mode = mode
print("t      z     vz    vbat   f_hold  mode  integ  base   z_ref | logged mean motor")
sw = next((j for j in range(1, len(rows)) if rows[j][5] != rows[j-1][5] and rows[j][5] == 0), None)
a, b = (max(0, sw - 40), min(len(rows), sw + 120)) if sw else (len(rows) - 100, len(rows))
for r in rows[a:b:5]: print(f"{r[0]:6.2f} {r[1]:5.2f} {r[2]:5.2f} {r[3]:5.2f}  {r[4]:5.3f}   {r[5]}   {r[6]:5.3f} {r[7]:5.3f} {r[8]:5.2f} |  {r[9]:5.3f}")
