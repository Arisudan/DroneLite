"""Flight recorder (Nimbus: flight recording): every connection writes docs/flights/YYYY-MM-DD_HHMMSS_<real-fc|simulator|bench-props-off>.csv (25 Hz rows) and .events.txt.
Rows: time, source/phase/state, commands (controller output and what was sent to the FC), range, flow, quality, attitude, battery, link, FC mode, safety latches,
controller estimate (height, reference) and, when the radio probe is on, motor outputs. Written from the bus, so it records Sim/Bench/live the same way."""
from __future__ import annotations

import csv
import math
import os
import time
from typing import Optional

from . import topics as T

DIR = os.environ.get("DRONEFORGE_FLIGHTS", os.path.expanduser("~/Music/DroneForge/docs/flights"))
FIELDS = ["t", "source", "phase", "state", "cmd_roll", "cmd_pitch", "cmd_thr", "cmd_arm", "sent_thr", "range_m", "flow_fwd", "flow_right", "flow_q",
          "pitch_deg", "roll_deg", "batt_v", "lq", "fc_mode", "safety_disarm", "safety_land", "z_est", "z_ref", "m1", "m2", "m3", "m4", "ramp_phase", "fc_x", "fc_y", "fc_z", "fc_vx", "fc_vy", "fc_vz", "fc_valid", "est_x", "est_y", "est_vx", "est_vy", "age_att_ms", "age_d3_ms", "age_d4_ms", "age_batt_ms"]


MAX_FLIGHTS = int(os.environ.get("DRONEFORGE_MAX_FLIGHTS", "300"))       # keep at most this many flight logs
MIN_FREE_M = float(os.environ.get("DRONEFORGE_MIN_FREE_MB", "2000"))      # ... and always leave this much free disk space (megabytes)


def _num(row: dict, key: str):
    try:
        return float(row[key]) if row.get(key, "") != "" else None
    except (ValueError, TypeError):
        return None


def _rms(v: list) -> float:
    return math.sqrt(sum(a * a for a in v) / len(v)) if v else float("nan")


def summarize(csv_path: str) -> str:
    """Plain-text flight summary, ALL distances in metres. Used when a log closes and by tools_scripts/flight_log_report.py."""
    with open(csv_path, newline="") as f:
        rows = list(csv.DictReader(f))
    out = [f"flight log: {os.path.basename(csv_path)}", f"rows: {len(rows)}"]
    if not rows:
        return "\n".join(out + ["no data recorded"])
    T_ = [_num(r, "t") or 0.0 for r in rows]
    out.append(f"duration: {T_[-1]:.1f} s")
    armed = [r for r in rows if r.get("cmd_arm") == "1"]
    out.append(f"armed: {(len(armed) / len(rows) * T_[-1]):.1f} s (commanded)")
    rng = [v for v in (_num(r, "range_m") for r in rows) if v is not None]
    if rng:
        out.append(f"range sensor height (m): min {min(rng):.3f}  max {max(rng):.3f}")
    bv = [v for v in (_num(r, "batt_v") for r in rows) if v]
    if bv:
        out.append(f"battery (V): start {bv[0]:.2f}  end {bv[-1]:.2f}  min {min(bv):.2f}")
    hov = [r for r in rows if r.get("state") == "hover"]
    if hov:
        ez = [(_num(r, "z_est") or 0) - (_num(r, "z_ref") or 0) for r in hov]
        ex = [_num(r, "est_x") for r in hov if _num(r, "est_x") is not None]; ey = [_num(r, "est_y") for r in hov if _num(r, "est_y") is not None]
        out.append(f"HOVER {_num(hov[-1], 't') - _num(hov[0], 't'):.1f} s: height error rms {_rms(ez):.3f} m, max {max(abs(a) for a in ez):.3f} m")
        if ex and ey:
            dx, dy = [a - ex[0] for a in ex], [b - ey[0] for b in ey]
            out.append(f"  position (our Kalman filter) moved from the hold point: rms {_rms([math.hypot(a, b) for a, b in zip(dx, dy)]):.3f} m, max {max(math.hypot(a, b) for a, b in zip(dx, dy)):.3f} m")
    mm = [[_num(r, k) for k in ("m1", "m2", "m3", "m4")] for r in armed]
    mm = [x for x in mm if None not in x and max(x) > 1010]
    if mm:
        top = max(range(len(mm)), key=lambda i: sum(mm[i]))
        out.append(f"motor outputs sent by the FC while armed (1000..2000): max per motor {[int(max(x[j] for x in mm)) for j in range(4)]}, last {[int(v) for v in mm[-1]]}")
    fcv = [r for r in rows if r.get("fc_valid") == "1"]
    if any(r.get("fc_valid", "") != "" for r in rows):
        out.append(f"FC EKF (0xD4): valid in {len(fcv)} of {len(rows)} rows ({len(fcv) / len(rows) * 100:.0f} %)")
        pairs = [(_num(r, "fc_x"), _num(r, "est_x"), _num(r, "fc_y"), _num(r, "est_y")) for r in fcv if r.get("state") in ("hover", "takeoff")]
        pairs = [p for p in pairs if None not in p]
        if len(pairs) > 5:
            out.append(f"  FC EKF vs our filter, horizontal velocity-free position difference rms (m): x {_rms([p[0] - p[1] for p in pairs]):.3f}  y {_rms([p[2] - p[3] for p in pairs]):.3f}")
        fz = [(_num(r, "fc_z"), _num(r, "range_m")) for r in fcv]; fz = [p for p in fz if None not in p]
        if fz:
            out.append(f"  FC EKF height vs range sensor rms (m): {_rms([a - b for a, b in fz]):.3f}  (max FC height {max(a for a, _ in fz):.3f} m)")
    for col, name in (("age_att_ms", "attitude"), ("age_d3_ms", "range+flow"), ("age_d4_ms", "FC estimate")):
        v = [_num(r, col) for r in rows if _num(r, col) is not None]
        if v:
            out.append(f"  telemetry age, {name}: max {max(v) / 1000:.2f} s")
    ev = csv_path[:-4] + ".events.txt"
    if os.path.exists(ev):
        lines = open(ev).read().strip().splitlines()
        out.append(f"events ({len(lines)}):"); out += ["  " + l for l in lines[:25]]
    return "\n".join(out)


def prune(keep_path: str = "") -> list:
    """Free space for the next log: while there are more than MAX_FLIGHTS logs or less than MIN_FREE_M megabytes free, delete the OLDEST flight (csv + events + summary)."""
    removed = []
    try:
        bases = sorted({os.path.join(DIR, n.split(".")[0]) for n in os.listdir(DIR) if n.startswith("flight_") or n.split(".")[0].endswith("_simulator")})   # only legacy 'flight_*' and simulator sessions are ever auto-deleted: real-fc and bench recordings never are
        import shutil
        while bases and (len(bases) > MAX_FLIGHTS or shutil.disk_usage(DIR).free / 1e6 < MIN_FREE_M):
            b = bases.pop(0)
            if keep_path and b == keep_path[:-4]:
                continue
            for ext in (".csv", ".events.txt", ".summary.txt"):
                try: os.remove(b + ext)
                except OSError: pass
            removed.append(os.path.basename(b))
    except OSError:
        pass
    return removed


class FlightRecorder:
    def __init__(self, core, rate_hz: float = 25.0) -> None:
        self.core, self.period = core, 1.0 / rate_hz
        self.path: Optional[str] = None
        self._f = self._w = self._ev = None
        self._t0 = 0.0
        self._next = 0.0
        self.rows = 0
        self.last: dict = {}
        self.probe = None                                  # optional RadioFcProbe (motor outputs)
        core.bus.subscribe(T.TELEMETRY, self._tel)
        core.bus.subscribe(T.CONTROLS, self._row)
        core.bus.subscribe(T.EVENT, self._event)
        core.bus.subscribe(T.CONNECTION, self._conn)
        self._arr: dict = {}
        core.bus.subscribe(T.FRAME_IN, self._frame)

    def _conn(self, m) -> None:
        if m.get("connected"):
            self.start()
        else:
            self.stop()

    def start(self) -> None:
        self.stop()
        try:
            os.makedirs(DIR, exist_ok=True)
            prune()                                          # make room BEFORE opening the new log
            src = getattr(self.core, "state_source", None)
            kind = "simulator" if src == "sim" else "bench-props-off" if src == "bench" else "real-fc"
            base = os.path.join(DIR, time.strftime("%Y-%m-%d_%H%M%S") + "_" + kind)       # e.g. 2026-11-09_153245_real-fc.csv (same names the docs reorganisation uses)
            self._f = open(base + ".csv", "w", newline=""); self._w = csv.DictWriter(self._f, FIELDS); self._w.writeheader()
            self._ev = open(base + ".events.txt", "w")
            self.path, self._t0, self._next, self.rows, self.last = base + ".csv", time.monotonic(), 0.0, 0, {}
        except OSError:
            self._f = self._w = self._ev = None          # a full disk or a bad path must never stop the link

    def stop(self) -> None:
        path = self.path if self._f else None
        for fh in (self._f, self._ev):
            try:
                if fh: fh.close()
            except OSError:
                pass
        self._f = self._w = self._ev = None
        if path and self.rows:
            try:
                with open(path[:-4] + ".summary.txt", "w") as f:
                    f.write(summarize(path) + "\n")
            except Exception:
                pass                                         # a summary problem must never disturb the link

    def _event(self, text: str) -> None:
        if self._ev:
            try:
                self._ev.write(f"{time.monotonic() - self._t0:8.2f}s  {text}\n"); self._ev.flush()
            except OSError:
                pass

    def _frame(self, f) -> None:
        self._arr[f.type] = time.monotonic()                 # arrival time of the last frame of each type: lets the log show WHICH frame went stale and for how long

    def _tel(self, msg) -> None:
        n, d = msg
        L = self.last
        if n == "optrange":
            L.update(range_m=d["distance_m"], flow_fwd=d["velocity_forward"], flow_right=d["velocity_right"], flow_q=d["flow_quality"])
        elif n == "attitude":
            L.update(pitch_deg=math.degrees(d["pitch_rad"]), roll_deg=math.degrees(d["roll_rad"]))
        elif n == "battery":
            L["batt_v"] = d["voltage_v"]
        elif n == "link":
            L["lq"] = d["up_lq"]
        elif n == "flight_mode":
            L["fc_mode"] = d["mode"]
            if "motors" in d:
                L["_motors"] = d["motors"]
        elif n == "kinematic":                                 # the FC's own IMU-rate EKF state (0xD4), logged next to ours to compare them in flight
            L.update(fc_x=d["x_m"], fc_y=d["y_m"], fc_z=d["z_m"], fc_vx=d["vx_mps"], fc_vy=d["vy_mps"], fc_vz=d["vz_mps"], fc_valid=int(d["valid"]))

    def _row(self, c) -> None:
        if not self._w:
            return
        now = time.monotonic() - self._t0
        if now < self._next:
            return
        self._next = now + self.period
        core, au, L = self.core, self.core.autonomy, self.last
        m = self.probe.motors if (self.probe and self.probe.motors) else (L.get("_motors") or [None] * 4)      # the FC's own motor outputs arrive in its flight-mode text (wireless)
        row = {"t": round(now, 3), "source": core.source, "phase": au.phase, "state": au.ctl.state.value if au.phase == "flying" else "",
               "cmd_roll": round(c.roll, 4), "cmd_pitch": round(c.pitch, 4), "cmd_thr": round(c.throttle, 4), "cmd_arm": int(c.arm),
               "sent_thr": round(core.last_sent.throttle, 4), "safety_disarm": core.safety.disarm_latched or "", "safety_land": core.safety.land_request or "",
               "z_est": round(au.ctl.z, 3) if au.phase == "flying" else "", "z_ref": round(au.ctl.z_ref, 3) if au.phase == "flying" else "",
               "m1": m[0], "m2": m[1], "m3": m[2], "m4": m[3], "ramp_phase": core.ramp.phase}
        ctl = au.ctl
        nowm = time.monotonic()
        for col, ft in (("age_att_ms", 0x1E), ("age_d3_ms", 0xD3), ("age_d4_ms", 0xD4), ("age_batt_ms", 0x08)):
            a = self._arr.get(ft)
            if ft == 0x1E:                                                   # the attitude age is the freshest of the ATTITUDE frame and the usable D4 frame (attitude_select)
                sel = getattr(core, "att_sel", None)
                t_sel = sel.last_time() if sel is not None else None
                if t_sel is not None and (a is None or t_sel > a):
                    a = t_sel
            row[col] = "" if a is None else int((nowm - a) * 1000)
        row.update(est_x=round(ctl.kx.x[0], 3), est_y=round(ctl.ky.x[0], 3), est_vx=round(ctl.kx.x[1], 3), est_vy=round(ctl.ky.x[1], 3))
        for k in ("fc_x", "fc_y", "fc_z", "fc_vx", "fc_vy", "fc_vz", "fc_valid"):
            v = L.get(k, "")
            row[k] = round(v, 3) if isinstance(v, float) else v
        for k in ("range_m", "flow_fwd", "flow_right", "flow_q", "pitch_deg", "roll_deg", "batt_v", "lq", "fc_mode"):
            v = L.get(k, "")
            row[k] = round(v, 4) if isinstance(v, float) else v
        try:
            self._w.writerow(row); self.rows += 1
            if self.rows % 50 == 0:
                self._f.flush()
        except OSError:
            pass
