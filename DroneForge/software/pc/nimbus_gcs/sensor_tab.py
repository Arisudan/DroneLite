"""SENSORS tab: does the MTF-02P data reach the FC, and does the FC's Kalman filter (EKF) actually use it?

Everything here comes over the radio from the FC itself, so it shows what the FC sees (Betaflight Configurator cannot show it):
  * frame 0xD3 is BUILT BY THE FC from the values it parsed from the sensor: distance, signal strength/precision, flow quality/status, flow velocity.
    If these change when you lift or slide the drone, the sensor -> FC link works.
  * frame 0xD4 is the FC EKF state (height, vertical speed, position, velocity). It is only meaningful if the filter is fed.
  * FUSION CHECK = compare the two over the last seconds: EKF height vs the range, EKF vertical speed vs the range's rate of change, EKF horizontal speed vs the flow speed.
    A filter that ignores the sensor would not follow it. To judge it the drone must MOVE: lift it gently by hand and slide it along the floor (props off or disarmed!).
Nothing here arms or commands anything.
"""
from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

from PyQt6.QtCore import Qt
from PyQt6.QtGui import QColor
from PyQt6.QtWidgets import QLabel, QVBoxLayout, QWidget

from . import theme
from .components import Card
from .widgets import ACCENT, GOOD, WARN, StripChart

WINDOW_S = 8.0
OK, WARNING, BAD, INFO = "ok", "warn", "bad", "info"
_COL = {OK: theme.ACC, WARNING: theme.AMB, BAD: theme.RED, INFO: theme.DIM}


def _pairs(a: List[Tuple[float, float]], b: List[Tuple[float, float]], t_min: float):
    da = {t: v for t, v in a}
    return [(da[t], v) for t, v in b if t in da and t >= t_min]


def _corr(xs: List[float], ys: List[float]) -> Optional[float]:
    n = len(xs)
    if n < 6:
        return None
    mx, my = sum(xs) / n, sum(ys) / n
    sx = math.sqrt(sum((x - mx) ** 2 for x in xs)); sy = math.sqrt(sum((y - my) ** 2 for y in ys))
    if sx < 1e-9 or sy < 1e-9:
        return None
    return sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / (sx * sy)


def fusion_report(h: Dict[str, List[Tuple[float, float]]], window_s: float = WINDOW_S) -> List[Tuple[str, str]]:
    """h: fresh-only series {fz_range, fz_ekf, fv_flow, fv_ekf, fvz_ekf} of (t, value). Returns [(level, text)]."""
    out: List[Tuple[str, str]] = []
    ends = [s[-1][0] for s in h.values() if s]
    if not ends:
        return [(INFO, "No sensor/EKF data yet (connect and power the drone).")]
    t_min = max(ends) - window_s
    # 1. height: EKF z vs the range sensor
    p = _pairs(h.get("fz_range", []), h.get("fz_ekf", []), t_min)
    if len(p) < 8:
        out.append((INFO, "Height: waiting for both the sensor range and the EKF height."))
    else:
        r = [a for a, _ in p]; z = [b for _, b in p]
        md = sum(abs(a - b) for a, b in p) / len(p); spread = max(r) - min(r); c = _corr(r, z)
        if md > 0.06:
            out.append((BAD, f"Height: the EKF height differs from the sensor by {md:.2f} m on average: the range is NOT being fused (or the sensor is rejected)."))
        elif spread >= 0.10 and c is not None:
            out.append((OK if c > 0.9 else WARNING, f"Height: the EKF follows the range sensor (mean difference {md:.3f} m, correlation {c:.2f} over a {spread:.2f} m movement)."))
        else:
            out.append((OK, f"Height: the EKF agrees with the range sensor (mean difference {md:.3f} m). Lift the drone gently by hand to prove it follows movement."))
    # 2. vertical speed: EKF vz vs d(range)/dt
    rs = h.get("fz_range", [])
    dr: List[Tuple[float, float]] = []
    for i in range(len(rs)):
        j = next((k for k in range(i + 1, len(rs)) if rs[k][0] - rs[i][0] >= 0.5), None)
        if j is not None and rs[i][0] >= t_min:
            dr.append((rs[i][0], (rs[j][1] - rs[i][1]) / (rs[j][0] - rs[i][0])))
    if len(dr) < 8:
        out.append((INFO, "Vertical speed: waiting for data."))
    else:
        # match EKF vz to the middle of each 0.5 s interval by nearest sample
        ek = h.get("fvz_ekf", [])
        xs, ys = [], []
        for t, v in dr:
            nearest = min(ek, key=lambda e: abs(e[0] - (t + 0.25)), default=None)
            if nearest and abs(nearest[0] - (t + 0.25)) < 0.2:
                xs.append(v); ys.append(nearest[1])
        mv = max((abs(x) for x in xs), default=0.0); c = _corr(xs, ys)
        if mv >= 0.08 and c is None:
            out.append((BAD, "Vertical speed: the range sensor shows vertical movement but the EKF vertical speed stays flat: the range is NOT being fused."))
        elif mv < 0.08 or c is None:
            out.append((INFO, "Vertical speed: not enough vertical movement to judge: lift the drone up and down gently by hand."))
        else:
            out.append((OK if c > 0.6 else BAD, f"Vertical speed: the EKF {'follows' if c > 0.6 else 'does NOT follow'} the range's rate of change (correlation {c:.2f})."))
    # 3. horizontal speed: EKF speed vs flow speed
    p = _pairs(h.get("fv_flow", []), h.get("fv_ekf", []), t_min)
    if len(p) < 8:
        out.append((INFO, "Horizontal speed: waiting for data."))
    else:
        f = [a for a, _ in p]; e = [b for _, b in p]; c = _corr(f, e)
        if max(f) >= 0.08 and c is None:
            out.append((BAD, "Horizontal speed: the optical flow shows movement but the EKF speed stays flat: the flow is NOT being fused."))
        elif max(f) < 0.08 or c is None:
            out.append((INFO, "Horizontal speed: the flow sensor saw no movement: slide the drone along the floor (texture needed under it) to prove the flow is fused."))
        else:
            ratio = (sum(e) / len(e)) / max(sum(f) / len(f), 1e-6)
            ok = c > 0.6
            out.append((OK if ok else BAD, f"Horizontal speed: the EKF {'follows' if ok else 'does NOT follow'} the optical flow (correlation {c:.2f}, EKF/flow speed ratio {ratio:.2f})."))
    return out


def _span(level: str, text: str) -> str:
    return f'<span style="color:{_COL[level]}">{text}</span>'


class SensorTab(QWidget):
    def __init__(self, win) -> None:
        super().__init__()
        self.win, self.model = win, win.model
        l = QVBoxLayout(self); l.setContentsMargins(0, 0, 0, 0); l.setSpacing(16)
        head = QLabel("MTF-02P -> FC -> Kalman filter. Move the drone gently (disarmed, props safe) and watch the numbers change.")
        head.setWordWrap(True); head.setStyleSheet(f"color:{theme.DIM}"); l.addWidget(head)
        self.sensor = QLabel(""); self.sensor.setTextFormat(Qt.TextFormat.RichText); self.sensor.setAlignment(Qt.AlignmentFlag.AlignTop)
        self.ekf = QLabel(""); self.ekf.setTextFormat(Qt.TextFormat.RichText); self.ekf.setAlignment(Qt.AlignmentFlag.AlignTop)
        from PyQt6.QtWidgets import QHBoxLayout
        row = QHBoxLayout(); row.setSpacing(16)
        c1, c2 = Card("Sensor to FC"), Card("FC Kalman filter"); c1.body.addWidget(self.sensor); c2.body.addWidget(self.ekf)
        row.addWidget(c1, 1); row.addWidget(c2, 1); l.addLayout(row)
        self.ch_h = StripChart("Height (m): sensor range vs EKF", [("fz_range", "sensor range", theme.qcolor(theme.DIM)), ("fz_ekf", "EKF z", theme.qcolor(theme.ACC))], " m", None, None, window_s=30.0)
        self.ch_v = StripChart("Horizontal speed (m/s): optical flow vs EKF", [("fv_flow", "flow", theme.qcolor(theme.DIM)), ("fv_ekf", "EKF", theme.qcolor(theme.ACC))], " m/s", None, None, window_s=30.0)
        self.ch_h.setMaximumHeight(320); self.ch_v.setMaximumHeight(320)
        row2 = QHBoxLayout(); row2.setSpacing(16); row2.addWidget(self.ch_h); row2.addWidget(self.ch_v); l.addLayout(row2, 1)
        self.verdict = QLabel(""); self.verdict.setTextFormat(Qt.TextFormat.RichText); self.verdict.setWordWrap(True); cv = Card("Is it fused?"); cv.body.addWidget(self.verdict); l.addWidget(cv)

    def refresh(self) -> None:
        m = self.model; st = m.state; d = st.decoded
        h = m.history
        def rate(ftype): return st.rate(st.in_times, ftype)
        def age(name):
            a = st.age(name); return None if a is None else a
        o = d.get("optrange"); k = d.get("kinematic")
        ao, ak = age("optrange"), age("kinematic")
        fresh_o = ao is not None and ao < 1.0; fresh_k = ak is not None and ak < 1.0
        lines = ["<b>1. SENSOR -> FC</b> <span style='color:" + theme.DIM + "'>(frame 0xD3, built by the FC from what it parsed)</span><br>"]
        lines.append(_span(OK if (fresh_o and rate(0xD3) >= 4) else (WARNING if fresh_o else BAD), f"frames from the FC: {rate(0xD3):.1f} Hz, age {ao:.1f} s" if ao is not None else "frames from the FC: none") + "<br>")
        if o and fresh_o:
            dist = o["distance_m"]
            lines.append(_span(OK if dist > 0 else BAD, f"range distance: {dist:.3f} m" + ("" if dist > 0 else "   (no valid distance)")) + "<br>")
            lines.append(_span(OK if o["dist_strength"] > 0 else WARNING, f"range signal strength: {o['dist_strength']}   precision {o['dist_precision']}") + "<br>")
            q = o["flow_quality"]
            lines.append(_span(OK if q >= 50 else (WARNING if q > 0 else BAD), f"flow quality: {q}   ({'good' if q >= 50 else 'weak: needs a textured, lit floor' if q > 0 else 'none'})") + "<br>")
            lines.append(_span(OK if o["flow_status"] == 1 else BAD, f"flow status: {o['flow_status']}   ({'valid' if o['flow_status'] == 1 else 'invalid'})") + "<br>")
            lines.append(f"flow velocity: forward {o['velocity_forward']:+.2f}  right {o['velocity_right']:+.2f} m/s<br>")
        else:
            lines.append(_span(BAD, "no sensor frame: the FC does not report any MTF-02P data (check the sensor cable and UART3 config)") + "<br>")
        self.sensor.setText("".join(lines))
        lines = ["<b>2. FC KALMAN FILTER</b> <span style='color:" + theme.DIM + "'>(frame 0xD4)</span><br>"]
        lines.append(_span(OK if (fresh_k and rate(0xD4) >= 4) else (WARNING if fresh_k else BAD), f"frames from the FC: {rate(0xD4):.1f} Hz, age {ak:.1f} s" if ak is not None else "frames from the FC: none") + "<br>")
        if k and fresh_k:
            lines.append(_span(OK if k["valid"] else BAD, f"EKF state valid: {'yes' if k['valid'] else 'NO (not initialised: arm once, or the sensor data is missing)'}") + "<br>")
            lines.append(f"height z: {k['z_m']:+.3f} m   vertical speed: {k['vz_mps']:+.2f} m/s<br>")
            lines.append(f"position x {k['x_m']:+.2f}  y {k['y_m']:+.2f} m (relative to where it was armed)<br>")
            lines.append(f"velocity x {k['vx_mps']:+.2f}  y {k['vy_mps']:+.2f} m/s<br>")
        else:
            lines.append(_span(BAD, "no EKF frame") + "<br>")
        self.ekf.setText("".join(lines))
        rep = fusion_report({kk: list(h[kk]) for kk in ("fz_range", "fz_ekf", "fv_flow", "fv_ekf", "fvz_ekf")})
        self.verdict.setText("<b>3. IS IT FUSED?</b><br>" + "<br>".join(_span(lv, tx) for lv, tx in rep))
        self.ch_h.set_data({"fz_range": list(h["fz_range"]), "fz_ekf": list(h["fz_ekf"])})
        self.ch_v.set_data({"fv_flow": list(h["fv_flow"]), "fv_ekf": list(h["fv_ekf"])})
        self.ch_h.update(); self.ch_v.update()
