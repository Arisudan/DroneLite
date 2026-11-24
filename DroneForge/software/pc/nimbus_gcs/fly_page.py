"""Fly page ('Mission control' look): SceneView with the flight-view PiP, mission timeline above the height plot on one time axis, and a 376 px right column
(hold-to-ARM then TAKE OFF, DISARM / LAND with disabled reasons, target height, pre-flight list), bottom strip of readouts.
The page flies ONLY through win.model.core.director; no dialogs: results go to the inline status line and to toasts (DISARM / LAND carry an Undo)."""
from __future__ import annotations

import math
import time

import numpy as np

from PyQt6.QtCore import QSize, Qt, QTimer
from PyQt6.QtGui import QImage
from PyQt6.QtWidgets import QHBoxLayout, QLabel, QSizePolicy, QVBoxLayout, QWidget

from . import theme as T
from .components import ActionButton, Card, HoldButton, StepField
from .components.hold_button import default_hold_ms
from .video_source import get_source, load_choice
from .viz.bottom_strip import BottomStrip
from .viz.height_plot import HeightPlot
from .viz.preflight_list import PreflightList
from .viz.scene_view import SceneView

ARM_PHASES = ("ready", "landed")
TAKEOFF_PHASES = ("ready", "landed", "armed")
LAND_PHASES = ("arming", "armed", "takeoff", "climbing", "holding")
FLYING_PHASES = ("takeoff", "climbing", "holding", "landing")


MODE_SUB = {"alt_hold": "height", "pos_hold": "height + position"}
MODE_ICON = {"alt_hold": "alt", "pos_hold": "pos"}
MODES = (("alt_hold", "ALT HOLD", "the flight controller holds the height; it drifts sideways"),
         ("pos_hold", "POS HOLD", "the flight controller holds the height and the position"))
MODE_LABEL = {m: lab for m, lab, _ in MODES}


def check_mark(ok: bool, blocking: bool) -> tuple[str, str]:
    """(glyph, level) of a pre-flight row; the glyph differs per state so it reads without colour."""
    if ok:
        return "✔", "ok"
    return ("✖", "blocking") if blocking else ("⚠", "caution")


def first_blocker(checks) -> str:
    for c in checks:
        if not c.ok and c.blocking:
            return f"{c.label}: {c.detail}" if c.detail else c.label
    return ""


def disabled_reason(phase: str, checks) -> str:
    """Why ARM / TAKE OFF cannot be used (or will be refused) right now; '' when nothing stops them."""
    if phase == "disconnected":
        return "Not connected: connect the radio (or Sim) first."
    if phase == "fault":
        return "Safety fault latched: press SAFE, then reconnect."
    if phase not in TAKEOFF_PHASES:
        return {"arming": "Arming: gyro calibration, keep the drone still.", "landing": "Landing in progress."}.get(phase, f"Flying ({phase}): LAND or SAFE to end the flight.")
    b = first_blocker(checks)
    return f"ARM / TAKE OFF will be refused: {b}" if b else ""


# fallback when a check carries no goto: check id -> (workspace, sub-tab)
FIX_TARGETS = {"rest_tilt": ("SETUP", "Calibration"), "range": ("SETUP", "Sensors"), "link": ("SETUP", "Radio"), "telemetry": ("SETUP", "Radio"),
               "battery": ("SETTINGS", "Settings"), "ceiling": ("SETTINGS", "Settings")}


def mmss(sec: float) -> str:
    sec = max(0, int(sec)); return f"{sec // 60}:{sec % 60:02d}"


class _SideCol(QWidget):
    """Right column: its minimum is the compact stack (the page re-fits the fixed heights to whatever room it gets), so stale large heights never block shrinking."""
    def minimumSizeHint(self):
        return QSize(T.RIGHT_COL_W, self.min_h)
    min_h = 520


class FlyPage(QWidget):
    def __init__(self, win) -> None:
        super().__init__()
        self.win = win
        self._sig = None; self._checks: list = []
        self._t0 = time.monotonic()
        self._last_phase = None
        self.setMinimumWidth(1000)
        self._compact: bool | None = None; self._video_started = False; self._last_vf = None
        # --- left: scene (+ PiP), timeline, plot
        self.scene = SceneView(); self.plot = HeightPlot()
        self.phase_lbl = QLabel("PHASE  SAFE"); self.phase_lbl.setFont(T.f_body(12)); self.phase_lbl.setStyleSheet(f"color:{T.DIM}")
        left = QVBoxLayout(); left.setSpacing(16); left.setContentsMargins(0, 0, 0, 0)
        IGN = QSizePolicy.Policy.Ignored
        for wdg in (self.scene, self.plot):         # share the height by weight only (not by size hints)
            wdg.setSizePolicy(QSizePolicy.Policy.Expanding, IGN)
        self.scene.setMinimumHeight(150); self.plot.setMinimumHeight(180)
        from .rviz_map_view import RvizMapView
        self.flymap = RvizMapView(win); self.flymap.setSizePolicy(QSizePolicy.Policy.Ignored, IGN); self.scene.setSizePolicy(QSizePolicy.Policy.Ignored, IGN)
        top = QHBoxLayout(); top.setSpacing(12); top.setContentsMargins(0, 0, 0, 0); top.addWidget(self.scene, 1); top.addWidget(self.flymap, 1)   # half video, half map (2026-11-23)
        left.addLayout(top, 55)
        tl = QVBoxLayout(); tl.setSpacing(0); tl.setContentsMargins(24, 0, 24, 0)
        tl.addWidget(self.phase_lbl); tl.addWidget(self.plot, 1)
        left.addLayout(tl, 62)
        # --- right column
        self.btn_main = HoldButton("ARM", "", "drone"); self.btn_main.setFixedHeight(96)
        self.btn_disarm = ActionButton("DISARM", "unlock", "Available once armed"); self.btn_disarm.setToolTip("DISARM  (key D)")
        self.btn_land = ActionButton("LAND", "download", "Available in flight"); self.btn_land.setToolTip("LAND  (key L)")
        row = QHBoxLayout(); row.setSpacing(8); row.setContentsMargins(0, 0, 0, 0); row.addWidget(self.btn_disarm); row.addWidget(self.btn_land)
        st0 = win.model.core.autonomy.settings
        self.height_field = StepField(min(st0.hover_height, st0.ceiling_m), 0.05, 0.25, max(0.25, st0.ceiling_m))
        self.height_card = Card("Target height"); self.height_card.body.addWidget(self.height_field)
        self.mode_btns = {}
        # FLIGHT MODE card (2026-11-24 redesign): a two-part selector ALT HOLD | POS HOLD, EXPLORE as its own toggle (the PC explorer steers), one line saying what the mode does
        self.mode_card = Card("Flight mode"); mv = self.mode_card.body; mv.setSpacing(8)
        from .components.mode_tile import ModeTile                                       # 3D keycap tiles (2026-11-24): raised, the active one pressed in with an accent trace
        mrow = QHBoxLayout(); mrow.setSpacing(8); mrow.setContentsMargins(0, 0, 0, 0)
        for m, lab, _ in MODES:
            b = ModeTile(lab, MODE_SUB[m], MODE_ICON[m]); b.setToolTip(next(e for mm, _l, e in MODES if mm == m).capitalize())
            b.clicked.connect(lambda _=False, mm=m: self.on_mode(mm))
            self.mode_btns[m] = b; mrow.addWidget(b, 1)
        self.btn_explore = ModeTile("EXPLORE", "explorer steers", "explore")
        self.btn_explore.setToolTip("EXPLORE: the PC explorer steers through the map while the FC holds the height")
        self.btn_explore.clicked.connect(self.on_explore); self._explore_sig = None      # the PC explorer steers (nimbus_core/steering.py)
        self.mode_text = QLabel(""); self.mode_text.setWordWrap(True); self.mode_text.setMinimumHeight(30); self.mode_text.setAlignment(Qt.AlignmentFlag.AlignTop); self.mode_text.setFont(T.f_body(11)); self.mode_text.setStyleSheet(f"color:{T.DIM}")
        mrow.addWidget(self.btn_explore, 1); self._mrow = mrow; mv.addLayout(mrow); mv.addWidget(self.mode_text)
        self._mode_sig = None
        self.why = QLabel(""); self.why.setWordWrap(True); self.why.setStyleSheet(f"color:{T.DIM}"); self.why.setFont(T.f_body(12))
        self.status = QLabel("Ready.", self); self.status.setVisible(False); self.status.setWordWrap(True); self.status.setFont(T.f_body(12)); self.status.setStyleSheet(f"color:{T.TXT}")
        self.preflight = PreflightList()
        side = _SideCol(); side.setFixedWidth(T.RIGHT_COL_W)
        sl = QVBoxLayout(side); sl.setContentsMargins(0, 0, 0, 0); sl.setSpacing(6)
        sl.addWidget(self.btn_main); sl.addLayout(row); sl.addWidget(self.why); sl.addWidget(self.height_card); sl.addWidget(self.mode_card); sl.addWidget(self.preflight, 1)
        self.strip = BottomStrip()
        self.left, self.side, self.sl = left, side, sl
        self.body = body = QHBoxLayout(); body.setContentsMargins(24, 16, 24, 16); body.setSpacing(24); body.addLayout(left, 1); body.addWidget(side, 0)
        left.addWidget(self.strip)                       # 2026-11-24: the readouts sit under the chart; the right column (pre-flight list) runs to the bottom
        root = QVBoxLayout(self); root.setContentsMargins(0, 0, 0, 0); root.setSpacing(0); root.addLayout(body, 1)
        # --- wiring
        self.btn_main.activated.connect(self.on_main)
        self.btn_disarm.clicked.connect(self.do_disarm)
        self.btn_land.clicked.connect(self.do_land)
        self.height_field.valueChanged.connect(self.on_height)
        self.preflight.row_clicked.connect(self._row_clicked)
        self.set_compact(False)
        self._vtimer = QTimer(self); self._vtimer.setInterval(5); self._vtimer.timeout.connect(self._video_tick)      # a new picture is drawn within 5 ms of its arrival

    # --- layout: elastic, with a compact mode for short windows -------------------------------------
    COMPACT_BELOW = 870                          # of the page's OWN height (the content area), not the window's

    def set_compact(self, on: bool) -> None:
        if on == self._compact:
            return
        self._compact = on
        self.btn_main.setFixedHeight(50 if on else 96)
        for b in (self.btn_disarm, self.btn_land):
            b.setFixedHeight(48 if on else 72)
        cl = self.height_card.layout(); cl.setContentsMargins(*((16, 6, 16, 6) if on else (24, 16, 24, 16))); cl.setSpacing(2 if on else 8)
        self.height_card.body.setSpacing(2 if on else 8)
        self.mode_text.setMinimumHeight(16 if on else 30)
        ml = self.mode_card.layout(); ml.setContentsMargins(*((16, 6, 16, 6) if on else (24, 16, 24, 16))); ml.setSpacing(2 if on else 8)      # small windows: one row
        self.mode_card.body.setSpacing(4 if on else 8)
        for b in list(self.mode_btns.values()) + [self.btn_explore]: b.set_compact(on)
        self.mode_text.setVisible(not on); self._explore_sig = None; self._mode_sig = None       # re-style / re-label for the new size
        self.preflight.set_compact(on); self.strip.set_compact(on)
        self.sl.setSpacing(4 if on else 6); self.left.setSpacing(8 if on else 16)
        self.body.setContentsMargins(24, 6 if on else 16, 24, 6 if on else 16)
        self.updateGeometry()

    def resizeEvent(self, e) -> None:
        super().resizeEvent(e)
        self.set_compact(self.height() < self.COMPACT_BELOW)
        self._fit_side()

    def _fit_side(self) -> None:
        """The right column fills the scene+timeline block: spare height goes first to the pre-flight rows (24..40 px), the rest to the action buttons / target height / mode."""
        sl = self.sl; m = self.body.contentsMargins()
        side_h = self.height() - m.top() - m.bottom()
        on = bool(self._compact)
        main0, act0, fld0 = (50, 50, 64) if on else (96, 72, 96)      # compact trimmed 2026-11-21: room for the vision bar under the pages
        mode_btn0 = 36 if on else 72
        why_h = self.why.heightForWidth(T.RIGHT_COL_W) + 0 if self.why.isVisibleTo(self) else 0
        n = max(1, len(self.preflight._rows))
        mode_txt0 = 16 if on else 30
        cm = self.height_card.layout().contentsMargins(); card_x = cm.top() + cm.bottom() + 20 + self.height_card.layout().spacing()
        mm = self.mode_card.layout().contentsMargins(); mode_x = mm.top() + mm.bottom() + 20 + (1 if on else 3) * self.mode_card.body.spacing()      # the mode card: margins, title, gaps
        mode_rows = mode_btn0 if on else (mode_btn0 + mode_txt0)                                                                              # ALT | POS | EXPLORE tiles in one row; compact: no text
        fixed = main0 + act0 + why_h + fld0 + card_x + mode_rows + mode_x + 2 + (6 if not on else 0) + sl.spacing() * (5 if why_h else 4)
        pf_min = self.preflight.min_height_for(24, False)
        self.side.min_h = int(fixed + pf_min)
        extra = max(0, side_h - fixed - pf_min)
        pf_x = min(extra * 0.5, 16 * n + 32); rest = extra - pf_x
        self.btn_main.setFixedHeight(int(main0 + rest * 0.34))
        for b in (self.btn_disarm, self.btn_land):
            b.setFixedHeight(int(act0 + rest * 0.24))
        fh = int(fld0 + rest * 0.26)
        self.height_field.setMinimumSize(300, fh); self.height_field.setMaximumHeight(fh)
        if not on: self.mode_text.setMinimumHeight(int(mode_txt0 + rest * 0.16))
        for b in list(self.mode_btns.values()) + [self.btn_explore]:
            b.setFixedHeight(int(mode_btn0 + min(10, rest * 0.03)))

    # --- live video on the scene (shared VideoSource, same instance as the VIDEO tab) ---------------------
    def showEvent(self, e) -> None:
        super().showEvent(e)
        if not self._video_started:
            self._video_started = True
            try:
                src = get_source(self.win)
                if load_choice()["kind"] != "none" and not src.running:
                    src.start()                                  # non-blocking: opens in the source's own thread
            except Exception:                                    # noqa: BLE001
                pass
        self._vtimer.start(); self._video_tick()

    def hideEvent(self, e) -> None:
        self._vtimer.stop(); super().hideEvent(e)

    def _video_tick(self) -> None:
        src = get_source(self.win); st = src.state
        f, t_read = src.latest_with_time() if st == "live" else (None, 0.0)
        if f is None:
            if self.scene.has_frame() and src.held_frame()[0] is not None:        # a short drop (Wi-Fi hiccup, reconnect): keep the last picture, say so; no blank / blink
                tag = f"VIDEO DROPPED \u00b7 {src.frame_age():.1f} s \u00b7 reconnecting"
                if tag != getattr(self, "_drop_tag", ""): self._drop_tag = tag; self.scene.set_video_tag(tag)
                return
            self._drop_tag = ""
            self._last_vf = None
            self._video_room(False)
            if self.scene.has_frame():
                self.scene.set_frame(None)
            self.scene.set_video_tag("VIDEO CONNECTING" if st == "connecting" else "VIDEO STALLED" if st == "stalled" else "VIDEO OFF")
            return
        if f is self._last_vf and time.monotonic() - getattr(self, "_tag_t", 0.0) < 0.2: return     # nothing new: the 5 ms poll costs nothing
        self._tag_t = time.monotonic()
        hs = src.health_snapshot()["state"]                      # the tag is an alert only: nothing on screen while the video is healthy
        if hs in ("LIVE", "STARTING"):
            self.scene.set_video_tag("")
        else:
            d = src.health_snapshot()
            self.scene.set_video_tag(f"VIDEO {hs} \u00b7 {d['age_s']:.1f} s" if hs in ("FROZEN", "NO SIGNAL") else f"VIDEO {hs} \u00b7 {d['fps']:.0f} fps")
        self._video_room(True)
        if f is self._last_vf:
            return
        self._last_vf = f
        vb = getattr(self.win, "vision_bar", None)
        if vb is not None: f = vb.engine.annotate(f)                       # vision overlays on the flight view too
        try:
            h, w = f.shape[:2]
            f = np.ascontiguousarray(f); img = QImage(f.data, w, h, 3 * w, QImage.Format.Format_BGR888).copy() if (f.ndim == 3 and f.shape[2] == 3) else None
        except Exception:                                        # noqa: BLE001
            img = None
        self.scene.set_frame(img)
        src.note_shown(t_read)                                   # display delay for the health line (Video page)

    def _video_room(self, live: bool) -> None:
        """A live FPV picture gets more of the page (55 %) than the placeholder scene (38 %); the height plot keeps the rest."""
        if live != getattr(self, "_video_big", None):
            self._video_big = live; self.left.setStretch(0, 55); self.left.setStretch(1, 45)      # the map shares the row: it keeps its room with or without video

    @property
    def director(self):
        return self.win.model.core.director

    # --- inline status + toasts ----------------------------------------------------------------
    def say(self, res, warn: bool = False, undo=None) -> None:
        self.status.setText(res.message)
        if hasattr(self.win, "say"):
            self.win.say(res.message, "ok" if res.ok and not warn else "warn" if res.ok else "err")
        toasts = getattr(self.win, "toasts", None)
        if toasts is not None and (undo is not None or not res.ok):
            toasts.show_toast(res.message, undo if res.ok else None)

    # --- commands -----------------------------------------------------------------------------
    def on_main(self) -> None:
        ph = self.director.state().phase
        if ph == "armed":
            self.say(self.director.takeoff(self.height_field.value()))
        elif ph in ARM_PHASES:
            self.say(self.director.arm())
        else:
            self.say(self.director.arm())                       # refused by the director with its own message

    def do_disarm(self) -> None:
        res = self.director.disarm()
        self.say(res, undo=None)

    def do_land(self) -> None:
        res = self.director.land()
        self.say(res, undo=None)

    def on_height(self, v: float) -> None:
        """In flight: hold the new height now. On the ground: only the take-off height (not saved)."""
        if self.director.state().phase in ("takeoff", "climbing", "holding"):
            res = self.director.set_height(v)
            self.say(res)
            if not res.ok:
                self.height_field.set_value(self.director.state().target_m)

    def on_mode(self, mode: str) -> None:
        res = self.director.set_mode(mode)
        if res.ok:
            self.status.setText(res.message)               # the description sits once, under the MODE buttons
        else:
            self.say(res)

    def on_explore(self) -> None:
        on = not getattr(self.win.model.core.autonomy, "explore", False)
        res = self.director.explore(on)
        self.say(res) if not res.ok else self.status.setText(res.message)
        self._explore_sig = None

    def _refresh_explore(self, ph: str) -> None:
        au = self.win.model.core.autonomy; on = bool(getattr(au, "explore", False)); steer = getattr(au, "steer", None)
        hz = steer.rate_hz() if (on and steer is not None and steer.running) else 0.0
        can = on or ph == "holding"
        sig = (on, can, round(hz))
        if sig == self._explore_sig:
            return
        self._explore_sig = sig
        b = self.btn_explore; b.setEnabled(can)
        b.setText("\u25cf EXPLORE" if on else "EXPLORE"); b.set_active(on)
        b.set_sub(f"steering {hz:.0f} Hz" if on else ("explorer steers" if can else "while holding"))
        b.setToolTip("EXPLORE: the PC explorer steers through the map while the FC holds the height" if can else "Available while holding in flight")
        if on:
            self.mode_text.setText(f"EXPLORE: the explorer steers ({hz:.0f} Hz), the FC holds the height" if hz >= 5 else
                                   "EXPLORE: waiting for the explorer: sticks centred, the FC holds the position")
            self.mode_text.setStyleSheet(f"color:{T.TXT if hz >= 5 else T.AMB}")
        else:
            self._mode_sig = None; self.mode_text.setStyleSheet(f"color:{T.DIM}")

    def _refresh_mode(self, cur: str, connected: bool) -> None:
        cur = cur if cur in MODE_LABEL else "pos_hold"
        sig = (cur, connected)
        if sig == self._mode_sig:
            return
        self._mode_sig = sig
        for m, lab, _ in MODES:
            b, on = self.mode_btns[m], m == cur
            b.setText(("● " if on else "") + lab); b.setEnabled(connected); b.set_active(on and connected)
        self.mode_card.set_meta(MODE_LABEL[cur] if connected else "")
        expl = next(e for m, _, e in MODES if m == cur)
        self.mode_text.setText(f"{MODE_LABEL[cur]}: {expl}" if connected else "Not connected")

    def _row_clicked(self, rid: str) -> None:
        c = next((c for c in self._checks if c.id == rid), None)
        if c is None or c.ok:
            return
        tgt = c.goto or FIX_TARGETS.get(rid)
        if tgt:
            self.win.goto(*tgt)
        elif c.fix_hint:
            self.win.say(c.fix_hint, "warn")

    # --- refresh (10 Hz, called by the window) ------------------------------------------------
    def refresh(self, s: dict) -> None:
        m, d = self.win.model, self.director
        st = d.state(); checks = d.preflight(); self._checks = list(checks)
        ph = st.phase; live = s["connected"]
        ceiling = m.core.autonomy.settings.ceiling_m
        if abs(self.height_field._max - max(0.25, ceiling)) > 1e-9:
            self.height_field.set_range(0.25, max(0.25, ceiling))
        # controls: ARM (hold) then TAKE OFF; DISARM / LAND with reasons
        why = disabled_reason(ph, checks)
        self.btn_main.set_label("TAKE OFF" if ph == "armed" else "ARM")
        if ph == "armed":
            self.btn_main.set_enabled_with_reason(True); self.btn_main.set_hint(f"PRESS AND HOLD {default_hold_ms() / 1000:.1f} s  \u00b7  OR HOLD T"); why = ""
        elif ph in ARM_PHASES and not why:
            self.btn_main.set_enabled_with_reason(True); self.btn_main.set_hint(f"PRESS AND HOLD {default_hold_ms() / 1000:.1f} s  \u00b7  OR HOLD A")
        else:
            self.btn_main.set_enabled_with_reason(False, why or "Not available")
        wsig = (why, ph == 'armed')
        self.why.setText(why); self.why.setVisible(bool(why))
        if wsig != getattr(self, '_wsig', None):
            self._wsig = wsig; self._fit_side()
        self.btn_disarm.set_enabled_with_reason(ph == "armed", "" if ph == "armed" else "Not connected" if ph == "disconnected" else "Available once armed")
        self.btn_land.set_enabled_with_reason(ph in LAND_PHASES, "" if ph in LAND_PHASES else "Not connected" if ph == "disconnected" else "Available in flight")
        self.height_field.set_enabled_state(ph != "disconnected" and ph != "fault")
        self._refresh_mode(st.flight_mode, ph != "disconnected")
        self._refresh_explore(ph)
        # pre-flight
        sig = tuple((c.id, c.ok, c.blocking, c.detail) for c in checks)
        if sig != self._sig:
            self._sig = sig
            rows = []
            for c in checks:
                _g, level = check_mark(c.ok, c.blocking)
                state = "ready" if c.ok else (c.detail or "check")
                rows.append((c.id, c.label, state, level))
            self.preflight.set_checks(rows); self._fit_side()
        # readouts
        dec = m.state.decoded
        o, k, a = dec.get("optrange"), dec.get("kinematic"), dec.get("attitude")
        z = st.height_m if st.height_m is not None else (k["z_m"] if (k and k.get("valid")) else None)
        z = z if (live and z is not None) else None
        vz = k["vz_mps"] if (k and k.get("valid") and live) else 0.0
        rng = o["distance_m"] if (o and live) else None
        hdg = math.degrees(a["yaw_rad"]) if (a and live) else 0.0
        self.scene.set_values(z or 0.0, hdg, vz)
        flying = ph in FLYING_PHASES
        mp = st.mission_phase
        self.phase_lbl.setText(f"PHASE  {str(mp).upper().replace('_', '-')}   {mmss(st.phase_elapsed_s)}")
        if live:
            self.plot.push(time.monotonic() - self._t0, st.target_m if flying else 0.0, z, rng)
        f = lambda x, spec: "—" if x is None else format(x, spec)
        tgt = st.target_m if flying else None; err = (z - tgt) if (z is not None and tgt is not None) else None
        self.strip.set_values({"HEIGHT": f(z, ".2f"), "TARGET": f(tgt, ".2f"), "ERROR": "—" if err is None else f"{err:+.3f}", "RANGE": f(rng, ".2f")})   # LQ, battery: top bar
        self.strip.set_caution("ERROR", err is not None and abs(err) > 0.05 and st.phase_elapsed_s > 3.0)
