#!/usr/bin/env python3
"""Live viewer + scenario runner: the REAL height-hold C code of the flashed v3 firmware (v3/libhh_v3.so, built from the v3 source snapshot) flying a simulated drone.
The PC does what it does in flight: it only sends the target height, the hover reference and LAND. Everything else (take-off creep, climb, hand-over, hold, landing) is the firmware code.
  python3 sim_viewer.py                 window, real time x SPEED (default 4), all scenarios in a row, results printed and saved
  HEADLESS=1 python3 sim_viewer.py      no window, as fast as possible
Limits (honest): the plant is a simulation fitted to the real flights (ground effect, delays, speed bias are estimates); the real FC hardware is NOT in this loop."""
import ctypes, math, os, sys, time
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
from fc_hold_sim import FcCore, Plant

SCENARIOS = [
    ("1  Floor take-off to 0.5 m, hold, land (ground effect, normal delays)", dict(hover=0.72, ge=0.25), 0.66, 0.5, 12.0, None, {}),
    ("2  On boxes (no ground effect, hover 0.79)", dict(hover=0.79, ge=0.0, ground_range=0.19), 0.66, 0.5, 12.0, None, {}),
    ("3  Like your real drone: ground effect 35%, speed bias, delays, target 0.5 m", dict(hover=0.69, ge=0.35, ge_h=0.2, vz_bias=0.2, sensor_delay=0.10, act_delay=0.08), 0.66, 0.5, 12.0, None, {}),
    ("4  Like your real drone, target 0.3 m", dict(hover=0.69, ge=0.35, ge_h=0.2, vz_bias=0.2, sensor_delay=0.10, act_delay=0.08), 0.66, 0.3, 12.0, None, {}),
    ("5  Hover reference far too low (0.60 vs real hover 0.78)", dict(hover=0.78, ge=0.25), 0.60, 0.5, 12.0, None, {}),
    ("6  Height steps 0.5 -> 0.9 -> 0.4 m", dict(hover=0.72, ge=0.25), 0.66, 0.5, 26.0, [(8.0, 0.9), (17.0, 0.4)], {}),
    ("7  Something holds the drone down near the floor (snagging tether)", dict(hover=0.69, ge=0.25, tether_g=0.08, tether_h=0.25), 0.66, 0.5, 12.0, None, {}),
    ("8  Battery fades in flight -> thrust limit -> gentle landing", dict(hover=0.70, ge=0.25, sag=0.03), 0.66, 0.5, 120.0, None, {}),
    ("9  Slow sensor and command path (0.15 s + 0.15 s)", dict(hover=0.70, ge=0.25, sensor_delay=0.15, act_delay=0.15), 0.66, 0.5, 12.0, None, {}),
    ("10 Noisy speed reading (sigma 0.12 m/s)", dict(hover=0.72, ge=0.25, vz_sigma=0.12), 0.66, 0.5, 12.0, None, {}),
]


class Run:
    def __init__(self, spec, seed=1):
        self.name, kw, self.ref, self.target, self.hold_s, self.steps, over = spec
        self.core = FcCore(os.path.join(HERE, "v3", "libhh_v3.so")); self.L = self.core.L; self.core.set(**over)
        self.pl = Plant(seed=seed, **kw); self.dt = 0.002; self.fc_dt = 0.02; self.t_next = 0.0; self.cmd = 0.0
        for _ in range(40): self.pl.step(self.dt, 0.0)
        gz = []
        for _ in range(30): self.pl.step(self.dt, 0.0); gz.append(self.pl.sensed()[0])
        self.ground = sum(gz) / len(gz)
        self.L.hh_takeoff(ctypes.c_float(self.ref), ctypes.c_float(self.target), ctypes.c_float(self.pl.v0))
        self.t_air = None; self.t_land = None; self.done = False; self.reason = ""; self.cur_target = self.target; self.stepi = 0
        self.zmax = 0.0; self.vzmax = 0.0; self.hold_err = []; self.hist = []; self.t_target = None; self.t_start = self.pl.t; self.events = []
        self.mode = 1; self.vz_meas = 0.0; self.z_read = 0.0
    def advance(self, sim_s):
        end = self.pl.t + sim_s
        while self.pl.t < end and not self.done:
            pl = self.pl; L = self.L
            if pl.t >= self.t_next:
                self.t_next += self.fc_dt
                rng, vzs = pl.sensed(); zf = rng - self.ground; self.z_read, self.vz_meas = zf, vzs
                vb = max(pl.v0 - pl.sag * pl.t, 9.0)
                self.cmd = L.hh_update(ctypes.c_float(self.fc_dt), ctypes.c_float(zf), ctypes.c_float(vzs), ctypes.c_float(vb))
                m = L.hh_mode()
                if m != self.mode: self.events.append((round(pl.t, 1), {0: "HOLD", 1: "TAKE-OFF", 2: "LAND"}[m])); self.mode = m
                if m == 0 and self.t_air is None: self.t_air = pl.t
                if self.t_air is not None and self.stepi == 0 and self.t_target is None and abs(pl.z - self.target) < 0.08: self.t_target = pl.t - self.t_start
                if self.steps and self.t_air is not None and self.stepi < len(self.steps) and pl.t > self.t_air + self.steps[self.stepi][0]:
                    self.cur_target = self.steps[self.stepi][1]; L.hh_set_target(ctypes.c_float(self.cur_target)); self.stepi += 1
                if self.t_air is not None and self.t_land is None and pl.t > self.t_air + self.hold_s and m != 2: L.hh_land(); self.t_land = pl.t
                if self.t_air is not None and self.t_land is None and pl.t > self.t_air + 9.0: self.hold_err.append(abs(pl.z - self.cur_target))
                if L.hh_fail(): self.done = True; self.reason = "take-off failed: the FC gave up at the throttle cap (stopped on the floor)"
                elif L.hh_landed() and self.cmd <= 0.051 and pl.z < 0.02: self.done = True; self.reason = "landed"
            pl.step(self.dt, self.cmd)
            self.zmax = max(self.zmax, pl.z); self.vzmax = max(self.vzmax, abs(pl.vz))
            if pl.crashed: self.done = True; self.reason = f"CRASH: hit the floor at {pl.max_impact:.1f} m/s"
            if pl.t - self.t_start > 150.0: self.done = True; self.reason = "timeout (150 s)"
        self.hist.append((self.pl.t - self.t_start, self.pl.z, self.cmd, self.pl.vz))
        if len(self.hist) > 4000: self.hist = self.hist[::2]
    def result(self):
        he = self.hold_err
        return dict(name=self.name, outcome=self.reason, target=self.target, zmax=round(self.zmax, 2), overshoot=round(max(self.zmax - max(self.target, *(s[1] for s in (self.steps or [(0, 0)]))), 0.0), 2),
                    hold_max=round(max(he), 2) if he else None, hold_rms=round((sum(e * e for e in he) / len(he)) ** 0.5, 3) if he else None, vz_max=round(self.vzmax, 2),
                    touchdown=round(self.pl.max_impact, 2), t_target=round(self.t_target, 1) if self.t_target else None, t_total=round(self.pl.t - self.t_start, 1), events=self.events)


def run_headless():
    out = []
    for sp in SCENARIOS:
        r = Run(sp)
        while not r.done: r.advance(0.5)
        out.append(r.result())
    return out


def report(results):
    lines = ["FC height hold, REAL v3 firmware code in a simulated drone", ""]
    for r in results:
        lines.append(f"{r['name']}")
        lines.append(f"    outcome: {r['outcome']}   target {r['target']} m, highest {r['zmax']} m (overshoot {r['overshoot']} m), hold error max {r['hold_max']} / rms {r['hold_rms']} m, "
                     f"max vertical speed {r['vz_max']} m/s, touchdown {r['touchdown']} m/s, reached target after {r['t_target']} s, total {r['t_total']} s")
        lines.append("    phases: " + ", ".join(f"{t}s {e}" for t, e in r["events"]))
    return "\n".join(lines)


if __name__ == "__main__":
    if os.environ.get("HEADLESS") == "1":
        res = run_headless(); print(report(res)); sys.exit(0)
    from PyQt6.QtCore import QTimer, Qt, QPointF, QRectF
    from PyQt6.QtGui import QColor, QFont, QPainter, QPen, QBrush, QPolygonF
    from PyQt6.QtWidgets import QApplication, QWidget
    SPEED = float(os.environ.get("SPEED", "4"))
    class View(QWidget):
        def __init__(self):
            super().__init__(); self.setWindowTitle("DroneForge: real v3 FC hold code in a simulated drone (live)"); self.resize(1100, 680)
            self.i = 0; self.run = Run(SCENARIOS[0]); self.results = []; self.pause_until = 0.0; self.finished = False
            self.t = QTimer(self); self.t.timeout.connect(self.tick); self.t.start(33)
        def tick(self):
            now = time.monotonic()
            if self.finished: self.update(); return
            if now < self.pause_until: self.update(); return
            if self.run.done:
                self.results.append(self.run.result()); self.i += 1
                if self.i >= len(SCENARIOS): self.finished = True; self.update(); QTimer.singleShot(5000, QApplication.quit); return
                self.run = Run(SCENARIOS[self.i]); self.pause_until = now + 0.8; self.update(); return
            self.run.advance(0.033 * SPEED)
            if self.run.done: self.pause_until = now + 3.0
            self.update()
        def paintEvent(self, _e):
            p = QPainter(self); p.setRenderHint(QPainter.RenderHint.Antialiasing); W, H = self.width(), self.height()
            p.fillRect(0, 0, W, H, QColor("#10151c")); r = self.run; pl = r.pl
            p.setPen(QColor("#e6e6e6")); p.setFont(QFont("Sans", 12, QFont.Weight.Bold)); p.drawText(14, 26, f"Scenario {r.name}")
            p.setFont(QFont("Monospace", 10)); p.setPen(QColor("#9fb3c8"))
            p.drawText(14, 48, f"sim time {pl.t - r.t_start:6.1f} s (x{SPEED:g})   height {pl.z:5.2f} m   speed {pl.vz:+5.2f} m/s   throttle {r.cmd:4.2f}   mode {({0:'HOLD',1:'TAKE-OFF',2:'LAND'}[r.mode])}   target {r.cur_target:.2f} m")
            # side view
            gx, gy, scale = 70, H - 70, 330.0
            p.fillRect(QRectF(0, gy, W * 0.48, 70), QColor("#2a2f36")); p.setPen(QPen(QColor("#5a6572"), 2)); p.drawLine(0, int(gy), int(W * 0.48), int(gy))
            ge_h = pl.ge_h if pl.ge else 0.0
            if ge_h: p.fillRect(QRectF(0, gy - ge_h * scale, W * 0.48, ge_h * scale), QColor(80, 130, 200, 40)); p.setPen(QColor("#6f8fb8")); p.drawText(8, int(gy - ge_h * scale - 4), "ground effect zone")
            ty = gy - r.cur_target * scale; p.setPen(QPen(QColor("#e0a030"), 1, Qt.PenStyle.DashLine)); p.drawLine(0, int(ty), int(W * 0.48), int(ty)); p.drawText(8, int(ty - 4), f"target {r.cur_target:.2f} m")
            for k in range(1, 6):
                yy = gy - k * 0.25 * scale
                p.setPen(QColor("#3a424d")); p.drawLine(0, int(yy), 12, int(yy)); p.drawText(16, int(yy + 4), f"{k * 0.25:.2f}")
            cx = W * 0.24; cy = gy - pl.z * scale - 14
            col = QColor("#59c27a") if r.mode == 0 else (QColor("#e0a030") if r.mode == 1 else QColor("#c36"))
            p.setPen(QPen(col, 3)); p.drawLine(int(cx - 40), int(cy), int(cx + 40), int(cy)); p.setBrush(QBrush(col)); p.drawEllipse(QPointF(cx - 40, cy - 4), 7, 3); p.drawEllipse(QPointF(cx + 40, cy - 4), 7, 3); p.drawRect(QRectF(cx - 12, cy - 8, 24, 12))
            fl = max(0.0, min(1.0, (r.cmd - 0.05) / 0.8)); p.setPen(Qt.PenStyle.NoPen); p.setBrush(QColor(255, 140, 0, 200)); p.drawRect(QRectF(cx - 8, cy + 4, 16, 6 + 30 * fl))
            # plots
            def plot(x0, y0, w, h, series, title, lo, hi, color, tgt=None):
                p.fillRect(QRectF(x0, y0, w, h), QColor("#161c25")); p.setPen(QColor("#9fb3c8")); p.drawText(int(x0 + 6), int(y0 + 14), title)
                if tgt is not None:
                    yy = y0 + h - (tgt - lo) / (hi - lo) * h; p.setPen(QPen(QColor("#e0a030"), 1, Qt.PenStyle.DashLine)); p.drawLine(int(x0), int(yy), int(x0 + w), int(yy))
                hist = r.hist
                if len(hist) < 2: return
                t1 = hist[-1][0]; t0 = max(0.0, t1 - 60.0); pts = []
                for h_ in hist:
                    if h_[0] < t0: continue
                    xx = x0 + (h_[0] - t0) / max(t1 - t0, 1.0) * w; yy = y0 + h - (series(h_) - lo) / (hi - lo) * h
                    pts.append(QPointF(xx, max(y0, min(y0 + h, yy))))
                p.setPen(QPen(color, 2)); p.drawPolyline(QPolygonF(pts))
            px = W * 0.5; pw = W * 0.47
            plot(px, 70, pw, 170, lambda h: h[1], "height (m)", 0.0, 1.2, QColor("#59c27a"), r.cur_target)
            plot(px, 255, pw, 150, lambda h: h[2], "throttle (what the FC commands)", 0.0, 0.9, QColor("#e0a030"))
            plot(px, 420, pw, 150, lambda h: h[3], "vertical speed (m/s)", -1.0, 1.0, QColor("#6aa9ff"))
            p.setPen(QColor("#9fb3c8")); p.setFont(QFont("Monospace", 10)); yy = 592
            for t_, e in r.events[-4:]: p.drawText(int(px), yy, f"{t_:6.1f} s  {e}"); yy += 16
            if r.done:
                p.setFont(QFont("Sans", 13, QFont.Weight.Bold)); p.setPen(QColor("#59c27a") if r.reason == "landed" else QColor("#e06060")); p.drawText(14, 78, f"RESULT: {r.reason}")
            if self.finished:
                p.fillRect(QRectF(W * 0.2, H * 0.35, W * 0.6, 90), QColor(10, 14, 20, 230)); p.setPen(QColor("#e6e6e6")); p.setFont(QFont("Sans", 14, QFont.Weight.Bold)); p.drawText(int(W * 0.2 + 20), int(H * 0.35 + 36), "All scenarios finished: results are in the terminal / chat."); p.setFont(QFont("Sans", 11)); p.drawText(int(W * 0.2 + 20), int(H * 0.35 + 64), "This window closes in 5 seconds.")
    app = QApplication(sys.argv); v = View(); v.show(); code = app.exec()
    print(report(v.results)); open(os.path.join(HERE, "results_v3_sim.txt"), "w").write(report(v.results)); sys.exit(0)
