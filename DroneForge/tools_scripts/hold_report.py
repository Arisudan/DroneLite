#!/usr/bin/env python3
"""Did the flight reach HOLDING and did the VO aid reach the FC? Reads a hold CSV (docs/status_logs/hold_live/*_hold.csv). Read-only; no radio, no drone.
usage: hold_report.py [file.csv]     (no argument: the newest one)"""
import csv, glob, os, sys

HOLD_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "docs", "status_logs", "hold_live")


def _num(v):
    try: return float(v)
    except (TypeError, ValueError): return None


def _flag(us):
    n = _num(us)
    return n is not None and n > 1900


def report(rows):
    """rows: dicts of one hold CSV -> dict of counts; only armed rows count."""
    armed = [r for r in rows if _num(r.get("sent_arm")) == 1 or str(r.get("sent_arm")).lower() == "true"]
    phases = []
    for r in armed:
        if r.get("phase") and (not phases or phases[-1] != r["phase"]): phases.append(r["phase"])
    out = dict(armed_rows=len(armed), phases=phases, reached_holding="holding" in phases)
    for name, sel in (("all_armed", armed), ("holding", [r for r in armed if r.get("phase") == "holding"])):
        out[name] = dict(rows=len(sel), sent=sum(_flag(r.get("sent_aid_valid_us")) for r in sel), at_fc=sum(_flag(r.get("fc_aid_valid_us")) for r in sel))
    return out


def verdict(r):
    if not r["armed_rows"]: return "no armed rows: this is not a flight"
    if not r["reached_holding"]: return "never reached HOLDING (phases: %s); aid at the FC in %d of %d armed rows" % (" > ".join(r["phases"]), r["all_armed"]["at_fc"], r["armed_rows"])
    h = r["holding"]
    if not h["sent"]: return "HOLDING reached, but the GCS never sent a valid aid (check vo_state / vo_msg)"
    if not h["at_fc"]: return "HOLDING reached and the aid was sent, but the FC read-back never showed it (check the ELRS switch mode, DIAGNOSTICS channel test)"
    return "HOLDING reached; aid sent in %d and seen at the FC in %d of %d holding rows" % (h["sent"], h["at_fc"], h["rows"])


def main(argv):
    files = argv or sorted(glob.glob(os.path.join(HOLD_DIR, "*_hold.csv")))[-1:]
    if not files: print("no hold CSV found"); return 1
    for f in files:
        with open(f, newline="") as fh: r = report(list(csv.DictReader(fh)))
        print(os.path.basename(f)); print("  " + verdict(r))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
