import sys; sys.path.insert(0, ".")
from fc_hold_sim import *
print("FC core on the fitted plant, FC-side delays (sensor 0.06 s, act 0.03, motor tau 0.06), take-off with the PC's hover ref 0.66")
for name, kw in (("floor hover .72", dict(hover=0.72)), ("floor hover .66", dict(hover=0.66)), ("floor hover .78", dict(hover=0.78)), ("boxes hover .79 no GE", dict(hover=0.79, ge=0.0, ground_range=0.19))):
    for ref in (0.60, 0.66, 0.72):
        res = []
        for seed in (1, 2, 3):
            core = FcCore(); r = fly(Plant(seed=seed, **kw), core, f_hover_ref=ref); res.append(r)
        bad = sum(1 for r in res if r["crashed"] or r["fail"])
        print(f"{name:22} ref {ref:.2f}: fail {bad}/3  zmax {max(r['zmax'] for r in res):.2f}  vzmax {max(r['vzmax'] for r in res):.2f}  hold max {max(r['hold_max'] for r in res):.2f} rms {max(r['hold_rms'] for r in res):.2f}  touchdown {max(r['touchdown'] for r in res):.2f}  base {res[0]['base']:.3f}")
