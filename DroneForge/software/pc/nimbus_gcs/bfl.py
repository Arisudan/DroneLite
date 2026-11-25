#!/usr/bin/env python3
"""Betaflight blackbox (.BFL) decoder, written from the public log format (data version 2), no third-party package.
Usage: python3 -m nimbus_gcs.bfl LOG.BFL [out.csv]      -> prints header facts, frame counts and sanity checks, optionally writes every main frame (I and P) to CSV.
Supports exactly the encodings Betaflight 4.5 uses: SIGNED_VB 0, UNSIGNED_VB 1, NEG_14BIT 3, TAG8_8SVB 6, TAG2_3S32 7, TAG8_4S16 8, NULL 9, TAG2_3SVARIABLE 10 (not seen yet:
it raises, never guesses). Predictors 0 ZERO, 1 PREVIOUS, 2 STRAIGHT_LINE, 3 AVERAGE_2, 4 MINTHROTTLE, 5 MOTOR_0, 6 INC, 8 1500, 9 VBATREF, 10 LAST_MAIN_FRAME_TIME, 11 MINMOTOR.
A frame is accepted only if the byte after it is a valid frame marker (or the end of the log): otherwise the decoder reports a corrupt frame and resyncs on the next marker."""
from __future__ import annotations

import csv
import struct
import sys
from typing import Dict, List, Optional, Tuple

MARKERS = {ord(c) for c in "IPESGH"}


class Reader:
    def __init__(self, data: bytes, pos: int = 0) -> None:
        self.d, self.p = data, pos

    def eof(self) -> bool:
        return self.p >= len(self.d)

    def byte(self) -> int:
        if self.p >= len(self.d):
            raise EOFError
        b = self.d[self.p]; self.p += 1
        return b

    def uvb(self) -> int:
        v, shift = 0, 0
        for _ in range(5):
            b = self.byte(); v |= (b & 0x7F) << shift
            if not b & 0x80:
                return v
            shift += 7
        raise ValueError("bad variable-byte integer")

    def svb(self) -> int:
        n = self.uvb()
        return (n >> 1) ^ -(n & 1)                      # zigzag


def sx(v: int, bits: int) -> int:
    m = 1 << (bits - 1)
    return (v & (m - 1)) - (v & m)


def tag2_3s32(r: Reader) -> List[int]:
    lead = r.byte(); sel = lead >> 6
    if sel == 0:
        return [sx(lead >> 4, 2), sx(lead >> 2, 2), sx(lead, 2)]
    if sel == 1:
        b = r.byte(); return [sx(lead, 4), sx(b >> 4, 4), sx(b, 4)]
    if sel == 2:
        b1 = r.byte(); b2 = r.byte(); return [sx(lead, 6), sx(b1, 6), sx(b2, 6)]
    out = []
    s = lead
    for _ in range(3):
        k = s & 3
        if k == 0:
            out.append(sx(r.byte(), 8))
        elif k == 1:
            a = r.byte(); b = r.byte(); out.append(sx(a | (b << 8), 16))
        elif k == 2:
            a = r.byte(); b = r.byte(); c = r.byte(); out.append(sx(a | (b << 8) | (c << 16), 24))
        else:
            a = r.byte(); b = r.byte(); c = r.byte(); d = r.byte(); out.append(sx(a | (b << 8) | (c << 16) | (d << 24), 32))
        s >>= 2
    return out


def tag8_4s16(r: Reader) -> List[int]:
    sel = r.byte(); nib = 0; buf = 0; out = []
    for _ in range(4):
        k = sel & 3
        if k == 0:
            out.append(0)
        elif k == 1:
            if nib == 0:
                buf = r.byte(); out.append(sx(buf >> 4, 4)); nib = 1
            else:
                out.append(sx(buf, 4)); nib = 0
        elif k == 2:
            if nib == 0:
                out.append(sx(r.byte(), 8))
            else:
                c1 = (buf << 4) & 0xFF; buf = r.byte(); c1 |= buf >> 4; out.append(sx(c1, 8))
        else:
            if nib == 0:
                a = r.byte(); b = r.byte(); out.append(sx((a << 8) | b, 16))
            else:
                a = r.byte(); b = r.byte(); out.append(sx(((buf & 0x0F) << 12) | (a << 4) | (b >> 4), 16)); buf = b
        sel >>= 2
    return out


def tag8_8svb(r: Reader, n: int) -> List[int]:
    if n == 1:
        return [r.svb()]
    h = r.byte(); out = []
    for _ in range(n):
        out.append(r.svb() if h & 1 else 0); h >>= 1
    return out


class Log:
    def __init__(self, path: str) -> None:
        self.data = open(path, "rb").read()
        self.hdr: Dict[str, str] = {}
        i = 0
        while self.data[i:i + 2] == b"H ":
            j = self.data.index(b"\n", i); k, _, v = self.data[i + 2:j].decode("latin1").partition(":"); self.hdr[k] = v; i = j + 1
        self.start = i
        ints = lambda key: [int(x) for x in self.hdr[key].split(",")]
        self.names = self.hdr["Field I name"].split(",")
        self.i_pred, self.i_enc = ints("Field I predictor"), ints("Field I encoding")
        self.p_pred, self.p_enc = ints("Field P predictor"), ints("Field P encoding")
        self.s_names = self.hdr.get("Field S name", "").split(",") if self.hdr.get("Field S name") else []
        self.s_enc = ints("Field S encoding") if self.s_names else []
        self.vbatref = int(self.hdr.get("vbatref", 0)); self.minthrottle = int(self.hdr.get("minthrottle", 0)); self.motor_output = [int(x) for x in self.hdr.get("motorOutput", "0,0").split(",")]
        self.ix = {n: k for k, n in enumerate(self.names)}
        self.rows: List[List[int]] = []; self.events: List[Tuple[int, str]] = []; self.slow: List[Tuple[int, List[int]]] = []
        self.corrupt = 0; self.counts = {"I": 0, "P": 0, "E": 0, "S": 0}

    # ---- frame bodies -----------------------------------------------------------------------
    def read_i(self, r: Reader) -> List[int]:
        n = len(self.names); v = [0] * n; i = 0
        while i < n:
            e = self.i_enc[i]
            if e == 0: v[i] = r.svb(); i += 1
            elif e == 1: v[i] = r.uvb(); i += 1
            elif e == 3: v[i] = -sx(r.uvb(), 14); i += 1
            elif e == 9: v[i] = 0; i += 1
            elif e == 6:
                j = i + 1
                while j < n and j < i + 8 and self.i_enc[j] == 6: j += 1
                v[i:j] = tag8_8svb(r, j - i); i = j
            elif e == 7: v[i:i + 3] = tag2_3s32(r); i += 3
            elif e == 8: v[i:i + 4] = tag8_4s16(r); i += 4
            else: raise ValueError(f"I encoding {e} not supported")
        for i in range(n):
            p = self.i_pred[i]
            if p == 0: pass
            elif p == 4: v[i] += self.minthrottle
            elif p == 5: v[i] += v[self.ix["motor[0]"]]
            elif p == 8: v[i] += 1500
            elif p == 9: v[i] += self.vbatref
            elif p == 11: v[i] += self.motor_output[0]
            else: raise ValueError(f"I predictor {p} not supported")
        return v

    def read_p(self, r: Reader, h1: List[int], h2: List[int], skipped: int) -> List[int]:
        n = len(self.names); raw = [0] * n; i = 0
        while i < n:
            e = self.p_enc[i]
            if e == 0: raw[i] = r.svb(); i += 1
            elif e == 1: raw[i] = r.uvb(); i += 1
            elif e == 3: raw[i] = -sx(r.uvb(), 14); i += 1
            elif e == 9: raw[i] = 0; i += 1
            elif e == 6:
                j = i + 1
                while j < n and j < i + 8 and self.p_enc[j] == 6: j += 1
                raw[i:j] = tag8_8svb(r, j - i); i = j
            elif e == 7: raw[i:i + 3] = tag2_3s32(r); i += 3
            elif e == 8: raw[i:i + 4] = tag8_4s16(r); i += 4
            else: raise ValueError(f"P encoding {e} not supported")
        v = [0] * n
        for i in range(n):
            p = self.p_pred[i]
            if p == 6: v[i] = h1[i] + 1 + skipped
            elif p == 1: v[i] = h1[i] + raw[i]
            elif p == 2: v[i] = 2 * h1[i] - h2[i] + raw[i]
            elif p == 3: v[i] = int((h1[i] + h2[i]) / 2) + raw[i]
            elif p == 0: v[i] = raw[i]
            else: raise ValueError(f"P predictor {p} not supported")
        return v

    def read_event(self, r: Reader) -> str:
        t = r.byte()
        if t == 0: return f"sync beep t={r.uvb()}"
        if t == 14: return f"logging resume iteration={r.uvb()} t={r.uvb()}"
        if t == 15: return f"DISARM reason={r.uvb()}"
        if t == 30: a = r.uvb(); b = r.uvb(); return f"flight mode flags {b:#x} -> {a:#x}"
        if t == 13:
            f = r.byte()
            return f"inflight adjustment fn={f & 127} " + (f"float {struct.unpack('<f', bytes(r.byte() for _ in range(4)))[0]}" if f & 128 else f"value {r.svb()}")
        if t == 255:
            s = bytes(r.byte() for _ in range(11)); return "end of log" if s.startswith(b"End of log") else "log end marker (odd text)"
        raise ValueError(f"unknown event {t}")

    # ---- whole log ----------------------------------------------------------------------------
    def decode(self) -> None:
        r = Reader(self.data, self.start); h1: Optional[List[int]] = None; h2: Optional[List[int]] = None; last_iter = -1
        while not r.eof():
            m = r.byte()
            if m not in MARKERS:
                self.corrupt += 1; continue
            frame_start = r.p
            try:
                if m == ord("I"):
                    v = self.read_i(r)
                elif m == ord("P"):
                    if h1 is None: raise ValueError("P before I")
                    skipped = 0                                         # the iteration jump after dropped frames is unknown without the log's P-interval pattern: INC uses +1+skipped, skipped = 0 here
                    v = self.read_p(r, h1, h2 if h2 is not None else h1, skipped)
                elif m == ord("E"):
                    ev = self.read_event(r); nxt_ok = r.eof() or r.d[r.p] in MARKERS or ev == "end of log"
                    if not nxt_ok: raise ValueError("bad event")
                    self.events.append((self.rows[-1][1] if self.rows else 0, ev)); self.counts["E"] += 1
                    if ev == "end of log": break
                    continue
                elif m == ord("S"):
                    vals = []; i = 0
                    while i < len(self.s_enc):
                        e = self.s_enc[i]
                        if e == 1: vals.append(r.uvb()); i += 1
                        elif e == 0: vals.append(r.svb()); i += 1          # INAV slow frames carry signed fields (wind, temperatures)
                        elif e == 7: vals.extend(tag2_3s32(r)); i += 3
                        else: raise ValueError(f"S encoding {e}")
                    self.slow.append((self.rows[-1][1] if self.rows else 0, vals)); self.counts["S"] += 1
                    if not (r.eof() or r.d[r.p] in MARKERS): raise ValueError("bad S frame end")
                    continue
                else:
                    raise ValueError("GPS frames not supported")
                if not (r.eof() or r.d[r.p] in MARKERS):
                    raise ValueError("frame not followed by a marker")
            except (ValueError, EOFError):
                self.corrupt += 1; r.p = frame_start; continue
            self.rows.append(v); self.counts["I" if m == ord("I") else "P"] += 1
            h2, h1 = (v, v) if m == ord("I") else (h1, v)

    def column(self, name: str) -> List[int]:
        k = self.ix[name]; return [r[k] for r in self.rows]


DISARM_REASONS = {0: "arming disabled", 1: "failsafe", 2: "throttle timeout", 3: "sticks", 4: "arm switch (commanded)", 5: "crash protection", 6: "runaway takeoff", 7: "GPS rescue", 8: "serial command", 255: "system"}


def summarize(lg: "Log") -> str:
    """Plain-text findings for a decoded log: what happened, in numbers (all from the log itself, nothing guessed)."""
    if not lg.rows:
        return "No frames could be decoded from this log."
    t = [x / 1e6 for x in lg.column("time")]; t0 = t[0]; rel = [x - t0 for x in t]
    mot = list(zip(*[lg.column(f"motor[{i}]") for i in range(4)]))
    lo, hi = lg.motor_output; span = max(hi - lo, 1)
    frac = [[(m - lo) / span for m in row] for row in mot]; mean = [sum(f) / 4 for f in frac]
    out = [f"Firmware: {lg.hdr.get('Firmware revision', '?')}  built {lg.hdr.get('Firmware date', '?')}   loop {lg.hdr.get('looptime', '?')} us   {len(lg.rows)} frames, {rel[-1]:.2f} s, {lg.corrupt} corrupt"]
    armed = next((k for k, m in enumerate(mean) if m > 0.05), None)
    if armed is None:
        out.append("The motors never left idle in this log (no flight)."); return "\n".join(out)
    out.append(f"Motors started at {rel[armed]:.2f} s; armed for {rel[-1] - rel[armed]:.1f} s.")
    # burst / step detection: largest rise of the mean motor output within 20 ms
    s0 = max(0, armed - 100); j = s0; worst = 0.0               # start a little BEFORE the first frame above idle: a step shows up as one frame
    for k in range(s0, len(mean)):
        while rel[k] - rel[j] > 0.02: j += 1
        worst = max(worst, mean[k] - mean[j])
    peak = max(max(f) for f in frac)
    out.append(f"Peak single motor output: {peak * 100:.0f} % of range; steepest rise of the mean motor output: {worst * 100:.0f} % within 20 ms" + ("   <-- A THROTTLE STEP / BURST" if worst > 0.25 else "   (smooth)"))
    n = len(frac); tail = frac[int(n * 0.6):]
    pm = [sum(f[i] for f in tail) / len(tail) for i in range(4)]
    out.append("Mean motor outputs over the last 40 % of the log (M1..M4): " + "  ".join(f"{x:.2f}" for x in pm) + f"   spread {max(pm) - min(pm):.2f}" + ("   <-- uneven motors (balance, a weak motor, or the controller compensating)" if max(pm) - min(pm) > 0.08 else ""))
    if "surfaceRaw" in lg.ix:
        rg = lg.column("surfaceRaw"); gnd = sorted(rg[:max(5, armed)])[len(rg[:max(5, armed)]) // 2]
        up = next((k for k in range(armed, n) if rg[k] >= gnd + 5), None)
        out.append(f"Range sensor: {gnd/100:.2f} m on the floor, max {max(rg)/100:.2f} m" + (f"; first rose 0.05 m above the floor at {rel[up]:.2f} s (mean motor output then {mean[up]:.2f})" if up is not None else "; the drone never left the floor"))
    if "vbatLatest" in lg.ix:
        vb = [x / 100 for x in lg.column("vbatLatest")]
        out.append(f"Battery as the FC reads it: mean {sum(vb) / len(vb):.2f} V, min {min(vb):.2f}, max {max(vb):.2f}")
    if "axisI[0]" in lg.ix:
        ic = [lg.column(f"axisI[{a}]") for a in range(3)]                  # columns once (not inside the loop)
        act = sum(1 for k in range(armed, n) if ic[0][k] or ic[1][k] or ic[2][k]) / max(n - armed, 1)
        out.append(f"PID I-term active in {act * 100:.0f} % of the armed frames" + ("   (zero: held on the floor, or stick at minimum)" if act < 0.05 else ""))
    if "gyroADC[0]" in lg.ix:
        gx, gy = lg.column("gyroADC[0]"), lg.column("gyroADC[1]")
        out.append(f"Largest roll / pitch rate: {max(abs(x) for x in gx) / 10:.0f} / {max(abs(x) for x in gy) / 10:.0f} deg/s")
    if all(f"debug[{i}]" in lg.ix for i in range(4)) and "surfaceRaw" in lg.ix:                      # firmware v4.7.4R + debug_mode DF_HOLD
        out.extend(hold_report(lg, armed, rel))
    for tt, e in lg.events:
        if e.startswith("DISARM"):
            try: code = int(e.split("=")[1]); e = f"{e}  ({DISARM_REASONS.get(code, 'unknown')})"
            except ValueError: pass
        out.append(f"event @ {(tt / 1e6 - t0):.2f} s: {e}")
    return "\n".join(out)


def hold_report(lg: "Log", armed: int, rel: List[float], min_up_cm: float = 15.0) -> List[str]:
    """The sideways hold from the FC log (debug_mode DF_HOLD: debug[0] EKF velocity forward mm/s, [1] right mm/s, [2] / [3] VO aid forward / right mm, -32768 = no fresh aid).
    Airborne = the range sensor at least min_up_cm above the floor. Returns the report lines (empty when the drone never flew)."""
    rg = lg.column("surfaceRaw"); n = len(rg); head = sorted(rg[:max(5, armed)]); gnd = head[len(head) // 2]
    d = [lg.column(f"debug[{i}]") for i in range(4)]
    air = [k for k in range(armed, n) if rg[k] >= gnd + min_up_cm]
    if len(air) < 50:
        return []
    spd = [((d[0][k] ** 2 + d[1][k] ** 2) ** 0.5) / 1000.0 for k in air]
    rms = (sum(v * v for v in spd) / len(spd)) ** 0.5
    secs = rel[air[-1]] - rel[air[0]]
    have = [d[2][k] != -32768 and d[3][k] != -32768 for k in air]
    lines = [f"Sideways hold (DF_HOLD, {secs:.1f} s airborne): EKF speed rms {rms:.3f} m/s, max {max(spd):.2f} m/s"]
    if not any(have):
        lines.append("  VO aid: none reached the hold (no aid sent, older than 0.3 s, ALT HOLD, or the aid path is not carried by the radio link)")
        return lines
    longest = 0.0; prev = have[0]; start = rel[air[0]]
    for k, h in zip(air, have):                                          # longest stretch without a fresh aid
        if h:
            if not prev: longest = max(longest, rel[k] - start)
            prev = True
        else:
            if prev: start = rel[k]
            prev = False
    if not prev: longest = max(longest, rel[air[-1]] - start)
    mag = [((d[2][k] ** 2 + d[3][k] ** 2) ** 0.5) / 1000.0 for k, h in zip(air, have) if h]
    lines.append(f"  VO aid: present in {100.0 * sum(have) / len(have):.0f} % of the airborne time, longest gap without it {longest:.1f} s, aid size rms {(sum(v * v for v in mag) / len(mag)) ** 0.5:.3f} m, max {max(mag):.2f} m")
    return lines


def _down(vals: List[float], n: int, maxn: int = 500) -> List[float]:
    """Mean of consecutive groups so that at most maxn points remain."""
    if len(vals) <= maxn:
        return list(vals)
    g = -(-len(vals) // maxn)
    return [sum(vals[i:i + g]) / len(vals[i:i + g]) for i in range(0, len(vals), g)]


def analyze(lg: "Log") -> dict:
    """Numeric findings for the Analysis page (pure, no Qt): {'ok', 'duration', 'arm_time', 'armed_s', 'series': {name: [..]}, 't': [..], 'findings': [(level, text)], ...}.
    Series are downsampled to at most 500 points on the shared time axis 't' (s from the start of the log)."""
    res: dict = {"ok": False, "duration": 0.0, "arm_time": None, "armed_s": 0.0, "t": [], "series": {}, "findings": [], "lift_off": None, "hover": None, "sag": None, "disarm": None}
    F = res["findings"]
    if lg.corrupt > 0:
        F.append(("warn", f"{lg.corrupt} corrupt frame(s) in the log (the decoder skipped them)."))
    if not lg.rows or "time" not in lg.ix:
        F.insert(0, ("bad", "No frames could be decoded from this log.")); return res
    try:
        t = [x / 1e6 for x in lg.column("time")]; t0 = t[0]; rel = [x - t0 for x in t]
        mot = list(zip(*[lg.column(f"motor[{i}]") for i in range(4)]))
        lo, hi = lg.motor_output; span = max(hi - lo, 1)
        frac = [[(m - lo) / span for m in row] for row in mot]; mean = [sum(f) / 4 for f in frac]
    except (KeyError, ValueError, ZeroDivisionError):
        F.insert(0, ("bad", "The log has no motor/time columns.")); return res
    n = len(rel); res["ok"] = True; res["duration"] = rel[-1]
    ser = {f"M{i + 1}": [f[i] for f in frac] for i in range(4)}; ser["mean"] = mean
    if "rcCommand[3]" in lg.ix:
        ser["throttle"] = [min(max((x - 1000) / 1000.0, 0.0), 1.0) for x in lg.column("rcCommand[3]")]
    rg = lg.column("surfaceRaw") if "surfaceRaw" in lg.ix else None
    if rg is not None: ser["range"] = [float(x) for x in rg]
    if "vbatLatest" in lg.ix: ser["vbat"] = [x / 100 for x in lg.column("vbatLatest")]
    for a, nm in ((0, "roll"), (1, "pitch")):
        if f"gyroADC[{a}]" in lg.ix: ser[f"gyro_{nm}"] = [x / 10 for x in lg.column(f"gyroADC[{a}]")]
        if f"axisI[{a}]" in lg.ix: ser[f"i_{nm}"] = [float(x) for x in lg.column(f"axisI[{a}]")]
    res["t"] = _down(rel, n); res["series"] = {k: _down(v, n) for k, v in ser.items()}
    for tt, e in lg.events:
        if e.startswith("DISARM"):
            try: code = int(e.split("=")[1]); res["disarm"] = f"{e}  ({DISARM_REASONS.get(code, 'unknown')})"
            except (ValueError, IndexError): res["disarm"] = e
    armed = next((k for k, m in enumerate(mean) if m > 0.05), None)
    if armed is None:
        F.append(("ok", "The motors never left idle in this log (no flight).")); 
        if res["disarm"]: F.append(("ok", "Disarm: " + res["disarm"]))
        return res
    res["arm_time"] = rel[armed]; res["armed_s"] = rel[-1] - rel[armed]
    F.insert(0, ("ok", f"Motors started at {rel[armed]:.2f} s; armed for {res['armed_s']:.1f} s."))
    s0 = max(0, armed - 100); j = s0; worst = 0.0
    for k in range(s0, n):
        while rel[k] - rel[j] > 0.02: j += 1
        worst = max(worst, mean[k] - mean[j])
    res["burst"] = worst
    F.append(("bad", f"Throttle step / burst: the mean motor output rose {worst * 100:.0f} % within 20 ms.") if worst > 0.25 else ("ok", f"Motor ramp is smooth (steepest rise {worst * 100:.0f} % within 20 ms)."))
    peak = max(max(f) for f in frac); res["peak"] = peak
    if peak >= 0.995: F.append(("warn", "A motor reached 100 % output (no headroom left)."))
    tail = frac[int(n * 0.6):] or frac
    pm = [sum(f[i] for f in tail) / len(tail) for i in range(4)]; spread = max(pm) - min(pm); res["spread"] = spread
    txt = "Motor spread over the last 40 % of the log: " + f"{spread:.2f} (M1..M4 " + " ".join(f"{x:.2f}" for x in pm) + ")"
    F.append(("bad" if spread > 0.2 else "warn" if spread > 0.12 else "ok", txt + ("  uneven motors" if spread > 0.12 else "")))
    up = None
    if rg is not None:
        base = rg[:max(5, armed)]; gnd = sorted(base)[len(base) // 2]; res["floor_cm"] = gnd
        air = [k for k in range(armed, n) if rg[k] >= gnd + 5]
        up = air[0] if air else None
        if up is not None:
            res["lift_off"] = mean[up]; F.append(("ok", f"Lift-off: range first rose 0.05 m at {rel[up]:.2f} s with mean motor output {mean[up]:.2f}."))
        else:
            F.append(("ok", f"The drone never left the floor (floor {gnd/100:.2f} m, max {max(rg)/100:.2f} m)."))
        if "axisI[0]" in lg.ix and air:
            ic = [lg.column(f"axisI[{a}]") for a in range(3)]
            if not any(ic[0][k] or ic[1][k] or ic[2][k] for k in air):
                F.append(("warn", "PID I-term never active while the range was above the floor + 0.05 m."))
        # hover fit: median mean-motor where the range barely changes over 0.5 s, only with >= 3 s airborne
        if air and sum(rel[k + 1] - rel[k] for k in air if k + 1 < n) >= 3.0:
            pts = []; m = 0
            for k in air:
                while rel[k] - rel[m] > 0.5: m += 1
                if abs(rg[k] - rg[m]) / max(rel[k] - rel[m], 1e-3) < 5.0 and k - m > 2: pts.append(mean[k])
            if pts:
                res["hover"] = sorted(pts)[len(pts) // 2]; F.append(("ok", f"Hover throttle fit: median mean motor output {res['hover']:.2f} (range nearly constant)."))
    if "vbatLatest" in lg.ix:
        vb = ser["vbat"]; res["sag"] = max(vb) - min(vb)
        F.append(("ok", f"Battery as the FC reads it: {min(vb):.2f} to {max(vb):.2f} V, sag {res['sag']:.2f} V."))
    F.append(("ok", "Disarm: " + res["disarm"]) if res["disarm"] else ("ok", "No disarm event recorded."))
    return res


def main() -> None:
    lg = Log(sys.argv[1]); lg.decode()
    t = lg.column("time"); it = lg.column("loopIteration")
    print(f"{sys.argv[1]}: {lg.hdr.get('Firmware revision')} built {lg.hdr.get('Firmware date')}  looptime {lg.hdr.get('looptime')} us  vbatref {lg.vbatref/100:.2f} V")
    print(summarize(lg)); print()
    print(f"frames: {lg.counts}  corrupt/resync events: {lg.corrupt}  events: {len(lg.events)}")
    if lg.rows:
        dur = (t[-1] - t[0]) / 1e6
        mono = sum(1 for a, b in zip(t, t[1:]) if b < a)
        print(f"time {t[0]/1e6:.3f}..{t[-1]/1e6:.3f} s = {dur:.2f} s of data;  non-monotonic time steps: {mono};  mean frame spacing {dur/max(len(t)-1,1)*1e6:.0f} us")
    for tt, e in lg.events[:20]: print(f"  event @ {tt/1e6:.3f} s: {e}")
    if len(sys.argv) > 2:
        w = csv.writer(open(sys.argv[2], "w", newline="")); w.writerow(lg.names); w.writerows(lg.rows); print("wrote", sys.argv[2], len(lg.rows), "rows")


if __name__ == "__main__":
    main()
