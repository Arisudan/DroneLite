import sys, os, json, itertools
from multiprocessing import Pool
sys.path.insert(0, ".")
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
HOV = float(os.environ.get("HOV", "0.72")); DLY = float(os.environ.get("DLY", "0.33")); TMAX = float(os.environ.get("TMAX", "0.85")); CFG = float(os.environ.get("CFG", "0.72"))
BOX = os.environ.get("BOX", "0") == "1"
def job(args):
    over, seed = args
    os.environ["OVER"] = repr(over)
    from nimbus_pc.tests import test_auto_takeoff as T
    from nimbus_core import settings as S
    from nimbus_pc.hover.sim import SimParams
    if BOX:
        T.plant = lambda var, d=0.0: SimParams(thrust_exp=2.0, batt_sag_per_s=0.005, range_low_bias_m=S.SENSOR_LOW_BIAS_M, ground_effect_bonus=0.0, hover_thr=HOV + 0.07, ground_range_m=0.19, batt_v0=13.8, ctrl_delay_s=DLY, tau_motor=0.06, optrange_hz=17.0)
    else:
        T.plant = lambda var, d=0.0: SimParams(thrust_exp=2.0, batt_sag_per_s=0.005, range_low_bias_m=S.SENSOR_LOW_BIAS_M, ground_effect_bonus=0.25, ground_effect_h_m=0.1, hover_thr=HOV, ground_range_m=0.07, batt_v0=13.8, ctrl_delay_s=DLY, tau_motor=0.06, optrange_hz=17.0)
    st_ = S.FlightSettings(hover_height=0.5, thr_hover0=CFG, thr_max=TMAX, measured=True, batt_ref_v=13.8)
    try:
        res, _ = T.fly("floor", seed, st=st_, land_at=28.0, duration=50.0)
    except Exception as e:
        return (json.dumps(over, sort_keys=True), seed, "EXC " + str(e)[:40], 9, 9, 9)
    m = res.metrics; rows = res.rows
    hov = [x["t"] for x in rows if x["state"] == "hover"]
    t0 = hov[0] if hov else None
    hold = max((abs(x["z"] - 0.5) for x in rows if t0 is not None and t0 + 4 <= x["t"] <= min(t0 + 18, 27.5)), default=9)
    zmax = max((x["z"] for x in rows if x["t"] < 28), default=0)
    return (json.dumps(over, sort_keys=True), seed, bool(m.get("crashed")) or not m.get("reached_hover"), round(hold, 2), round(zmax, 2), round(m.get("touchdown_speed_mps") or 0, 2))
if __name__ == "__main__":
    combos = json.loads(sys.argv[1])
    seeds = range(1, int(sys.argv[2]) + 1)
    jobs = [(c, s) for c in combos for s in seeds]
    with Pool(8) as p: out = p.map(job, jobs)
    by = {}
    for o in out: by.setdefault(o[0], []).append(o)
    for k, v in by.items():
        bad = sum(1 for x in v if x[2] is True or isinstance(x[2], str))
        print(f"{k:90} fail {bad}/{len(v)} hold-err max {max(x[3] for x in v):.2f} zmax {max(x[4] for x in v):.2f} touchdown max {max(x[5] for x in v):.2f}", flush=True)
