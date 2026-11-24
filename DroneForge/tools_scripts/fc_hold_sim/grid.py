import os, sys, itertools, json; sys.path.insert(0, ".")
from multiprocessing import Pool
from fc_hold_sim import *
def job(a):
    kw, ref, seed, over = a
    core = FcCore(); core.set(**over)
    r = fly(Plant(seed=seed, **kw), core, f_hover_ref=ref, target=float(os.environ.get('TARGET', 0.5)))
    return (json.dumps(kw, sort_keys=True), ref, seed, r["crashed"], r["fail"], r["zmax"], r["vzmax"], r["hold_max"], r["touchdown"], r["landed"])
VARS = []
for hover in (0.64, 0.70, 0.78):
    for ge, rg in ((0.25, 0.07), (0.0, 0.19)):
        for sd, ad in ((0.04, 0.03), (0.10, 0.08), (0.15, 0.15)):
            VARS.append(dict(hover=hover, ge=ge, ground_range=rg, sensor_delay=sd, act_delay=ad))
VARS.append(dict(hover=0.69, ge=0.25, tether_g=0.08, tether_h=0.25))
# the real drone (11-13 15:00 tethered run): lift-off at 0.60 with a hover of 0.69, ground effect over the first ~0.2 m, FC speed bias +0.2 m/s near the floor
REAL2 = [dict(hover=0.69, ge=ge, ge_h=gh, vz_bias=b, vz_bias_h=bh, sensor_delay=sd, act_delay=ad) for ge, gh in ((0.35, 0.2), (0.45, 0.25)) for b, bh in ((0.12, 0.25), (0.2, 0.25)) for sd, ad in ((0.05, 0.04), (0.10, 0.08))]
REAL = [dict(hover=0.69, ge=0.35, ge_h=0.2, vz_bias=b, sensor_delay=sd, act_delay=ad) for b in (0.0, 0.2) for sd, ad in ((0.05, 0.04), (0.10, 0.08))]
if __name__ == "__main__":
    over = json.loads(sys.argv[1]) if len(sys.argv) > 1 else {}
    which = VARS + REAL if len(sys.argv) < 3 else (REAL if sys.argv[2] == 'real' else (REAL2 if sys.argv[2] == 'real2' else VARS))
    jobs = [(kw, ref, seed, over) for kw in which for ref in (0.66,) for seed in (1, 2, 3)]
    with Pool(8) as p: out = p.map(job, jobs)
    bad = [o for o in out if o[3] or o[4] or not o[9] or o[5] < 0.8 * float(os.environ.get('TARGET', 0.5))]
    print(f"{len(out)} runs, {len(bad)} failed (crash, take-off fail or not landed); zmax {max(o[5] for o in out):.2f} vz max {max(o[6] for o in out):.2f} hold max {max(o[7] for o in out if o[7]<9):.2f} touchdown max {max(o[8] for o in out):.2f}")
    for o in bad[:14]: print("  FAIL", o[0], "ref", o[1], "crash" if o[3] else "", "tkfail" if o[4] else "", "notlanded" if not o[9] else "", f"zmax {o[5]:.2f}")
