"""Cross-check: C frame builder from the Betaflight patch (host harness) vs nimbus_pc Python decoder."""
import subprocess, sys
import pathlib; sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[3] / "software" / "pc"))
from nimbus_pc import crsf

# (label, mm, quality, healthy, validX, validY, rateX rad/s, rateY rad/s, flowQuality, expected dict)
cases = [
    ("normal 1.500 m, forward 2 rad/s... scaled", 1500, 255, 1, 1, 1, 2.0, -0.5, 64, dict(dist=1.5, strength=255, fx=200, fy=-50, q=64, status=1)),
    ("close 59 mm", 59, 200, 1, 1, 1, 0.0, 0.0, 33, dict(dist=0.059, strength=200, fx=0, fy=0, q=33, status=1)),
    ("max valid 5999 mm", 5999, 10, 1, 1, 1, 0.0, 0.0, 10, dict(dist=5.999, strength=10, fx=0, fy=0, q=10, status=1)),
    ("out of range 6000 mm", 6000, 10, 1, 1, 1, 0.1, 0.1, 10, dict(dist=0.0, strength=10, fx=10, fy=10, q=10, status=1)),
    ("no data yet (negative mm)", -3, 0, 1, 1, 1, 0.0, 0.0, 0, dict(dist=0.0, strength=0, fx=0, fy=0, q=0, status=1)),
    ("zero mm", 0, 0, 1, 1, 1, 0.0, 0.0, 0, dict(dist=0.0, strength=0, fx=0, fy=0, q=0, status=1)),
    ("flow axis invalid -> zeros, status 0", 800, 100, 1, 1, 0, 1.0, 1.0, 50, dict(dist=0.8, strength=100, fx=0, fy=0, q=0, status=0)),
    ("flow unhealthy -> zeros, status 0", 800, 100, 0, 1, 1, 1.0, 1.0, 50, dict(dist=0.8, strength=100, fx=0, fy=0, q=0, status=0)),
    ("flow clamps to int16", 1000, 100, 1, 1, 1, 1000.0, -1000.0, 80, dict(dist=1.0, strength=100, fx=32767, fy=-32768, q=80, status=1)),
    ("quality clamps to u8", 1000, 100, 1, 1, 1, 0.0, 0.0, 999, dict(dist=1.0, strength=100, fx=0, fy=0, q=255, status=1)),
]
stdin = "\n".join(f"{c[1]} {c[2]} {c[3]} {c[4]} {c[5]} {c[6]} {c[7]} {c[8]}" for c in cases) + "\n"
out = subprocess.run(["./harness"], input=stdin, capture_output=True, text=True, check=True).stdout.split()
assert len(out) == len(cases)
bad = 0
for (label, *_, exp), hexframe in zip(cases, out):
    raw = bytes.fromhex(hexframe)
    p = crsf.FrameParser(); frames = p.feed(raw)
    ok = len(frames) == 1 and p.crc_errors == 0 and frames[0].type == crsf.TYPE_OPTRANGE and len(raw) == 14
    d = crsf.decode_optrange(frames[0].payload) if ok else None
    if d:
        checks = [abs(d["distance_m"] - exp["dist"]) < 1e-9, d["dist_strength"] == exp["strength"], d["dist_precision"] == 0,
                  d["flow_x_raw"] == exp["fx"], d["flow_y_raw"] == exp["fy"], d["flow_quality"] == exp["q"], d["flow_status"] == exp["status"]]
        ok = all(checks)
    print(("PASS " if ok else "FAIL ") + f"{label:42s} {hexframe}")
    bad += (not ok)
print("\nall frames: sync=0xC8 len=0x0C type=0xD3, 14 bytes, CRC-8/0xD5 valid" if not bad else f"\n{bad} FAILED")
sys.exit(1 if bad else 0)
