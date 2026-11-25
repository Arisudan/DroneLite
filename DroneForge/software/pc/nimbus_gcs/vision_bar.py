"""Bottom bar of the GCS: the vision engines (DEPTH, DETECT, FACE REC, TRACK, VO) as latching buttons under every page, one inline status line, and the
VO position-aid loop for the v4.7 FC (10 Hz). Layout after the DRONE_GCS footer bar; colours from theme.py only (one accent; amber for caution)."""
from __future__ import annotations

import math
import time

from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtWidgets import QHBoxLayout, QLabel, QLineEdit, QPushButton, QWidget

from . import theme as T
from .vision_engine import ENGINES, LABELS, VisionProcess
from .video_source import get_source
from .fc_caps import AidSupport

TIPS = {"depth": "Depth Anything 3 (metric) on the live video: colour overlay and the nearest distance",
        "detect": "Obstacles nearer than 1.0 m from the depth (6 x 8 cells); with models/detect.onnx people and objects",
        "face": "Face recognition: SCRFD detection + ArcFace against the saved faces (type a name and press Save face)",
        "track": "Follow one target: drag a box on the Video page, or it locks onto the recognised / largest face",
        "vo": "SuperPoint + LightGlue visual odometry: while hovering, the drift from the key frame goes to the FC (v4.7 position aid)"}
AID_HZ = 10.0
CALM_SPEED = 0.15            # m/s, as the firmware's brake-then-hold (xy_hold_core.c brake_speed)
CALM_S = 0.2
ANCHOR_MAX_WAIT_S = 2.0      # in HOLDING but never calm (a drifting drone is never slower than CALM_SPEED): set the key frame anyway after this long, the FC then pulls back to it
MSP_AID_PERIOD_S = 0.5       # the MSP copy of the aid (for v4.7 / v4.7.1 FCs): 2 Hz is all ELRS passes anyway; v4.7.2 reads the RC channels


class VisionBar(QWidget):
    def __init__(self, win) -> None:
        super().__init__()
        self.win = win; self.setObjectName("visionBar")
        self.engine = VisionProcess(lambda: get_source(win).latest_with_time())      # the models run in their own process: the video and the GUI stay at 30 fps
        h = QHBoxLayout(self); h.setContentsMargins(24, 6, 24, 6); h.setSpacing(10)
        lab = QLabel("VISION"); lab.setFont(T.f_label(10)); lab.setStyleSheet(f"color:{T.DIM};letter-spacing:2px"); h.addWidget(lab); h.addSpacing(6)
        self.btns = {}
        for k in ENGINES:
            b = QPushButton(LABELS[k]); b.setCheckable(True); b.setFixedHeight(30); b.setMinimumWidth(78); b.setFont(T.f_body(12)); b.setCursor(Qt.CursorShape.PointingHandCursor); b.setToolTip(TIPS[k])
            b.toggled.connect(lambda on, kk=k: self._toggle(kk, on)); self.btns[k] = b; h.addWidget(b)
        self.name = QLineEdit(); self.name.setPlaceholderText("name"); self.name.setFixedWidth(120); self.name.setFixedHeight(30)
        self.btn_enroll = QPushButton("Save face"); self.btn_enroll.setFixedHeight(30); self.btn_enroll.clicked.connect(self._enroll)
        h.addWidget(self.name); h.addWidget(self.btn_enroll)
        self.note = QLabel(""); self.note.setFont(T.f_body(11)); self.note.setStyleSheet(f"color:{T.DIM}"); self.note.setMinimumWidth(200)
        h.addWidget(self.note, 1)
        self.setStyleSheet(f"#visionBar {{ background:{T.PANEL}; border-top:1px solid {T.HAIR}; }}")
        self._style(); self._t_calm = None; self._t_hold = None; self._auto_vo = False; self._last_aid = 0.0; self.aid_sent = 0; self._msg = ""; self.aid_fw = AidSupport()      # does this FC firmware take the aid (v4.7 ST)? asked of the FC
        self._timer = QTimer(self); self._timer.timeout.connect(self._tick); self._timer.start(int(1000 / AID_HZ))

    # ---- buttons
    def _toggle(self, k: str, on: bool) -> None:
        self._msg = self.engine.set(k, on); self._style()

    def _enroll(self) -> None:
        self._msg = self.engine.enroll(self.name.text())

    def _style(self) -> None:
        for k, b in self.btns.items():
            on = b.isChecked(); b.setText(("● " if on else "") + LABELS[k])
            b.setStyleSheet(f"QPushButton{{background:{T.ACC if on else 'transparent'};color:{T.ON_ACC if on else T.TXT};border:1px solid {T.ACC if on else T.HAIR2};"
                            f"border-radius:6px;padding:0 14px;font-weight:{700 if on else 400};}}")
        face = self.btns["face"].isChecked(); self.name.setVisible(face); self.btn_enroll.setVisible(face)

    # ---- 10 Hz: status line + the VO position aid for the FC
    def _auto_start_vo(self) -> None:
        """VO runs by default (owner, 2026-11-25): once connected and the video is live it is switched on, once per session; the button still switches it off and that stays off."""
        if self._auto_vo or self.btns["vo"].isChecked(): return
        try:
            frame, _t = get_source(self.win).latest_with_time()
            if self.win.model.connected and frame is not None:
                self._auto_vo = True; self.btns["vo"].setChecked(True)          # the toggle starts the engine
        except Exception: pass                                                  # noqa: BLE001

    def _tick(self) -> None:
        self._auto_start_vo()
        e = self.engine; parts = []; warn = False
        dec = self.win.model.state.decoded; o = dec.get("optrange") or {}; att = dec.get("attitude") or {}
        if e.any_on() and o.get("distance_m", 0.0) > 0.2 and att:                    # DA3 floor scale: camera height (range, camera ~0.04 m above the sensor) + FC attitude (sign UNVERIFIED on the real FC)
            e.set_pose(o["distance_m"] + 0.04, att.get("pitch_rad", 0.0), att.get("roll_rad", 0.0))
        for k in ENGINES:
            if e.on[k]:
                st = e.state[k]; warn |= st == "error" or "weak" in e.text[k] or "cannot" in e.text[k]
                hz = f" {e.hz[k]:.0f} Hz" if st == "on" and e.hz[k] else ""
                parts.append(f"{LABELS[k]}{hz}: {e.text[k] or st}")
        if e.on["vo"]: parts.append(self._aid()); warn |= self.aid_fw.blocked
        if getattr(e, "msg", ""): self._msg = e.msg; e.msg = ""
        if self._msg and ("Saved" in self._msg or "No face" in self._msg or "Type a name" in self._msg): parts.insert(0, self._msg)
        txt = "   \u00b7   ".join(p for p in parts if p) or "Vision off. The models run on this PC on the live video."
        core = self.win.model.core
        core.vo_status = {"anchor": bool(e.on["vo"] and e.has_anchor()), "fresh": bool(e.on["vo"] and e.fresh_aid() is not None), "text": (parts[-1] if (e.on["vo"] and parts) else "")}
        self.note.setText(txt); self.note.setStyleSheet(f"color:{T.AMB if warn else (T.TXT if parts else T.DIM)}")

    def _aid(self) -> str:
        """Key frame when the drone holds still; the drift from it goes to the FC as MSP2_DF_POS_AID while it hovers (not in EXPLORE: the explorer steers then)."""
        core = self.win.model.core; e = self.engine
        if self.aid_fw.update(getattr(core, "sess", None)) != "yes":            # older firmware (rev1) or no answer: never send the aid
            if hasattr(core, "clear_pos_aid"): core.clear_pos_aid()
            if e.has_anchor(): e.clear_anchor()
            self._t_calm = None
            return self.aid_fw.text
        try:
            phase = core.director.state().phase; exploring = bool(getattr(core.autonomy, "explore", False))
        except Exception:                                             # noqa: BLE001
            return "position aid: no flight state"
        hovering = phase == "holding" and not exploring
        if not hovering and hasattr(core, "clear_pos_aid"): core.clear_pos_aid()
        self._t_hold = (self._t_hold or time.time()) if hovering else None
        fm = self.win.model.state.decoded.get("flight_mode") or {}
        if not hovering and str(fm.get("mode", "")).endswith("*"):            # BENCH: disarmed, carried by hand: show the VO displacement, NEVER send it (aid sign check)
            return self._bench_vo()
        if not hovering:
            if e.has_anchor(): e.clear_anchor()
            self._t_calm = None
            return "position aid: waiting for a hover"
        kin = self.win.model.state.decoded.get("kinematic") or {}
        spd = math.hypot(kin.get("vx_mps", 0.0) or 0.0, kin.get("vy_mps", 0.0) or 0.0)
        if not e.has_anchor():
            if spd < CALM_SPEED: self._t_calm = self._t_calm or time.time()
            else: self._t_calm = None
            if self._t_calm and time.time() - self._t_calm >= CALM_S and not e.anchor_pending(): e.request_anchor()
            elif self._t_hold and time.time() - self._t_hold >= ANCHOR_MAX_WAIT_S and not e.anchor_pending(): e.request_anchor()     # drifting: anchor anyway
            return "position aid: setting the key frame"
        aid = e.fresh_aid()
        sess = getattr(core, "sess", None)
        if aid is None or sess is None or not hasattr(sess, "send_frame"):
            if hasattr(core, "clear_pos_aid"): core.clear_pos_aid()
            return "position aid: no fresh VO (the FC holds on its flow)"
        if hasattr(core, "set_pos_aid"): core.set_pos_aid(aid[0], aid[1])            # v4.7.2: on RC channels CH13-15, in every ELRS packet (needs ELRS "100Hz Full")
        now = time.monotonic()
        if now - self._last_aid >= MSP_AID_PERIOD_S:                                 # v4.7 / v4.7.1: the MSP write, slowly (ELRS passes ~1 in 10 at 10 Hz)
            from nimbus_pc.commands import encode_msp_pos_aid
            for fr in encode_msp_pos_aid(aid[0], aid[1], True): sess.send_frame(fr)
            self._last_aid = now
        self.aid_sent += 1
        return f"position aid to the FC: {aid[0]:+.2f} / {aid[1]:+.2f} m"

    def _bench_vo(self) -> str:
        e = self.engine; kin = self.win.model.state.decoded.get("kinematic") or {}
        spd = math.hypot(kin.get("vx_mps", 0.0) or 0.0, kin.get("vy_mps", 0.0) or 0.0)
        if not e.has_anchor():
            if spd < CALM_SPEED: self._t_calm = self._t_calm or time.time()
            else: self._t_calm = None
            if self._t_calm and time.time() - self._t_calm >= CALM_S and not e.anchor_pending(): e.request_anchor()
            return "bench (disarmed, nothing sent): hold the drone still to set the VO start point"
        aid = e.fresh_aid()
        if aid is None: return "bench (disarmed, nothing sent): no fresh VO (point the camera at a textured scene)"
        return f"bench (disarmed, nothing sent): VO moved forward {aid[0]:+.2f} m, right {aid[1]:+.2f} m since the start point (VO off + on = new start point)"

    def shutdown(self) -> None:
        self._timer.stop(); self.engine.stop()
