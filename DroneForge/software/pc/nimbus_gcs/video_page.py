"""VIDEO workspace: the DJI O4 feed from CosmoStreamer (RTSP) with its health line, the latency test, the RTAB-Map 2D map (rviz_map_view, half of the page) and the snapshot.
The older depth-server floor plan / accumulated map and the first RTAB-Map view were removed on 2026-11-23 (archive/gcs_removed_2026-11-23).
The analog VTX settings, the VRX channel scan and the USB / analog sources were removed on 2026-11-21 (not part of the camera-flight pipeline; the previous page is archived in
archive/gcs_removed_2026-11-21). No dialogs, no check boxes, no yes/no questions: every outcome is an inline colour-coded status line. Nothing here arms or flies."""
from __future__ import annotations

import os
import time

from PyQt6.QtCore import QRectF, QSize, Qt, QTimer
from PyQt6.QtGui import QColor, QPainter
from PyQt6.QtWidgets import QHBoxLayout, QLabel, QLineEdit, QPushButton, QSizePolicy, QWidget

from .video_pip import frame_to_pixmap
from .video_health import format_line
from .video_latency import LatencyProbe, clock_ms, render_code
from .video_source import get_source, load_choice
from . import theme
from .components import Card

ACCENT, BAD, DIM, GOOD, WARN = theme.ACC, theme.RED, theme.DIM, theme.TXT, theme.AMB   # ok text = primary, caution amber, alarm red
DEFAULT_URL = "rtsp://192.168.50.1:554/video"                                            # CosmoStreamer RTSP server
# 2026-11-23: one dropdown to switch between the simulator and the real drone (video address + the radio port to connect to)
SIM_RADIO = os.path.expanduser("~/.config/droneforge/sim_radio")
PRESETS = [("sim", "Simulator", "udp://0.0.0.0:3000", "sim"),
           ("o4_rtsp", "DJI O4 \u00b7 CosmoStreamer (direct)", DEFAULT_URL, "dongle"),
           ("custom", "Custom address", "", None)]
PRESET_FILE = os.path.expanduser("~/.config/droneforge/video_preset.json")


def load_preset() -> str:
    try:
        import json; k = json.load(open(PRESET_FILE)).get("preset")
        return k if any(p[0] == k for p in PRESETS) else ""
    except Exception:                                                     # noqa: BLE001
        return ""


def save_preset(key: str) -> None:
    try:
        import json; os.makedirs(os.path.dirname(PRESET_FILE), exist_ok=True); json.dump({"preset": key}, open(PRESET_FILE, "w"))
    except OSError:
        pass

SNAP_DIR = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "docs", "video_snapshots"))



def _card(title: str, meta: str = ""):
    c = Card(title, meta)
    return c, c.body


def _status(label: QLabel, text: str, color: str) -> None:
    label.setText(text); label.setFont(theme.f_body(13)); label.setStyleSheet(f"color:{QColor(color).name()};")


class VideoView(QWidget):
    """Live frame (aspect kept) with an on-image status line."""
    def __init__(self) -> None:
        super().__init__()
        self.frame = None; self.text = ""; self.setMinimumHeight(260)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

    def set_frame(self, frame, text: str) -> None:
        self.frame = frame; self.text = text; self.update()

    # drag a box on the picture: TRACK follows it (vision_bar)
    on_box = None
    _drag = None

    def _to_frame(self, pos):
        if self.frame is None: return None
        fh, fw = self.frame.shape[:2]; s = min(self.width() / fw, self.height() / fh)
        ox, oy = (self.width() - fw * s) / 2, (self.height() - fh * s) / 2
        return ((pos.x() - ox) / s, (pos.y() - oy) / s)

    def mousePressEvent(self, e) -> None:
        self._drag = (e.position(), e.position()); self.update()

    def mouseMoveEvent(self, e) -> None:
        if self._drag: self._drag = (self._drag[0], e.position()); self.update()

    def mouseReleaseEvent(self, e) -> None:
        d, self._drag = self._drag, None; self.update()
        if not d or self.on_box is None: return
        a, b = self._to_frame(d[0]), self._to_frame(d[1])
        if a and b and abs(b[0] - a[0]) > 8 and abs(b[1] - a[1]) > 8:
            self.on_box((min(a[0], b[0]), min(a[1], b[1]), max(a[0], b[0]), max(a[1], b[1])))

    def paintEvent(self, _e) -> None:
        p = QPainter(self); p.fillRect(self.rect(), QColor(theme.BG))
        if self.frame is not None:
            pm = frame_to_pixmap(self.frame, self.size())
            p.drawPixmap((self.width() - pm.width()) // 2, (self.height() - pm.height()) // 2, pm)
        if self._drag:
            p.setPen(QColor(theme.ACC)); a, b = self._drag; p.drawRect(int(min(a.x(), b.x())), int(min(a.y(), b.y())), int(abs(b.x() - a.x())), int(abs(b.y() - a.y())))
        p.setPen(QColor(DIM if self.frame is None else theme.TXT))
        r = self.rect().adjusted(10, 8, -10, -8)
        p.drawText(r, Qt.AlignmentFlag.AlignCenter if self.frame is None else (Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignBottom),
                   self.text if self.frame is not None else (self.text or "No video"))


class LatencyCodeWindow(QWidget):
    """Shows a QR code that carries this PC's own clock in ms, redrawn every ~16 ms. The drone's camera is pointed at it; video_latency.py reads the code back out of the video and gives the delay."""
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("DroneForge latency test"); self.resize(720, 800)
        self._timer = QTimer(self); self._timer.setInterval(16); self._timer.timeout.connect(self.update)

    def showEvent(self, e) -> None:
        super().showEvent(e); self._timer.start()

    def hideEvent(self, e) -> None:
        self._timer.stop(); super().hideEvent(e)

    def paintEvent(self, _e) -> None:
        now_ms = clock_ms()                                   # taken at the start of the paint: the screen shows it one refresh later (8-17 ms)
        p = QPainter(self); p.fillRect(self.rect(), Qt.GlobalColor.white)
        side = min(self.width(), self.height() - 96) - 24
        if side > 60:
            pm = frame_to_pixmap(render_code(now_ms, scale=8), QSize(side, side))
            p.drawPixmap((self.width() - pm.width()) // 2, 12, pm)
        p.setPen(Qt.GlobalColor.black)
        p.drawText(QRectF(12, self.height() - 84, self.width() - 24, 72), Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignTop | Qt.TextFlag.TextWordWrap,
                   "Point the drone's camera at this code so that it fills most of the picture, and keep it steady. The GCS reads the code back out of the video; the delay appears in the Video window.")


class CalibBoardWindow(QWidget):
    """The checkerboard for 'Calibrate lens', as large as the window (maximise it): the O4 is pointed at this screen from different places and angles."""
    def __init__(self) -> None:
        super().__init__()
        from .lens_calib import board_image
        self.setWindowTitle("DroneForge lens calibration board"); self.resize(1100, 800); self.board = board_image()

    def paintEvent(self, _e) -> None:
        p = QPainter(self); p.fillRect(self.rect(), Qt.GlobalColor.white)
        pm = frame_to_pixmap(self.board, QSize(self.width() - 24, self.height() - 84))
        p.drawPixmap((self.width() - pm.width()) // 2, 12, pm); p.setPen(Qt.GlobalColor.black)
        p.drawText(QRectF(12, self.height() - 68, self.width() - 24, 60), Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignTop | Qt.TextFlag.TextWordWrap,
                   "Maximise this window. Point the O4 at it from different places and angles, the whole board in the picture, and hold still each time. "
                   "The Video page says where to go next. The screen must be flat: a printed board works too (tools_scripts/o4_calib/checkerboard_9x6.png).")


class VideoPage(QWidget):
    def __init__(self, win) -> None:
        super().__init__()
        self.win = win
        self.src = get_source(win)
        root = QHBoxLayout(self); root.setContentsMargins(0, 0, 0, 0); root.setSpacing(theme.SP["m"])
        p, pv = _card("Video")
        row = QHBoxLayout(); row.setSpacing(8)
        self.url = QLineEdit(); self.url.setPlaceholderText("rtsp://192.168.50.1:554/video  (CosmoStreamer: turn on Enable RTSP server first)")
        self.btn_start = QPushButton("Start"); self.btn_start.setObjectName("primary"); self.btn_stop = QPushButton("Stop"); self.btn_snap = QPushButton("Snapshot"); self.btn_lat = QPushButton("Latency test"); self.btn_cal = QPushButton("Calibrate lens")
        self.btn_start.clicked.connect(self.start_video); self.btn_stop.clicked.connect(self.stop_video)
        self.btn_snap.clicked.connect(self.snapshot); self.btn_lat.clicked.connect(self.toggle_latency_test); self.btn_cal.clicked.connect(self.toggle_calibration); self.cal = None; self.cal_window = None
        self.probe = LatencyProbe(lambda ms: self.src.health.on_e2e(ms)); self.lat_window = None; self._shown_t = None
        from PyQt6.QtWidgets import QComboBox
        self.preset = QComboBox(); self.preset.setMinimumWidth(240)
        for key, label, _url, _radio in PRESETS: self.preset.addItem(label, key)
        self.preset.setToolTip("Simulator or the real drone: sets the video address and the radio port (press Connect for the radio)")
        lab_src = QLabel("SOURCE"); lab_src.setStyleSheet(f"color:{theme.DIM}")
        row.addWidget(lab_src); row.addWidget(self.preset)
        row.addWidget(self.url, 1); row.addWidget(self.btn_start); row.addWidget(self.btn_stop); row.addWidget(self.btn_snap); row.addWidget(self.btn_lat); row.addWidget(self.btn_cal); pv.addLayout(row)
        self.view = VideoView()
        self.view.on_box = self._track_box
        from .rviz_map_view import RvizMapView
        self.rvmap = RvizMapView(win)   # 2026-11-23 (owner): ONE map, drawn exactly as RViz (DRONE_GCS layout + palettes), inside the GCS
        for wdg in (self.view, self.rvmap): wdg.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Expanding)
        views = QHBoxLayout(); views.setSpacing(theme.SP["m"]); views.addWidget(self.view, 1); views.addWidget(self.rvmap, 1); pv.addLayout(views, 1)   # half video, half map
        drow = QHBoxLayout(); drow.setSpacing(8)
        self.btn_map = QPushButton("Start map"); self.btn_map_stop = QPushButton("Stop map"); self.btn_map_clear = QPushButton("Clear map")
        self.btn_map.clicked.connect(self.start_map); self.btn_map_stop.clicked.connect(self.stop_map); self.btn_map_clear.clicked.connect(self.clear_map)
        drow.addStretch(1); drow.addWidget(self.btn_map); drow.addWidget(self.btn_map_stop); drow.addWidget(self.btn_map_clear); pv.addLayout(drow)
        self.video_card = p
        self.video_status = QLabel(""); self.video_status.setWordWrap(True); pv.addWidget(self.video_status)
        self.cal_status = QLabel(""); self.cal_status.setWordWrap(True); self.cal_status.hide(); pv.addWidget(self.cal_status)      # lens calibration: its own line (the video line is rewritten every 200 ms)
        self.frame = self.view                              # kept for older callers
        root.addWidget(p, 1)
        ch = load_choice(); self.url.setText(ch["target"] if ch["kind"] == "network" and ch["target"] else DEFAULT_URL)
        self._sync_preset(load_preset()); self.preset.activated.connect(self._on_preset)
        self.src.set_source("network", self._target(), analog=False)
        self._timer = QTimer(self); self._timer.timeout.connect(self._tick); self._timer.start(200)
        self._vtimer = QTimer(self); self._vtimer.timeout.connect(self._view_tick); self._vtimer.start(5)      # a NEW picture is drawn within 5 ms of its arrival (33 ms polling added up to a frame of delay)
        self._tick()

    # ---- helpers
    def _sync_preset(self, key: str = "") -> None:
        """Show the preset that matches the address (the saved one when it matches, else the first that does, else Custom)."""
        url = self.url.text().strip(); keys = [k for k, _l, u, _r in PRESETS if u == url]
        k = key if key in keys else (keys[0] if keys else "custom")
        self.preset.blockSignals(True); self.preset.setCurrentIndex(next(i for i, p in enumerate(PRESETS) if p[0] == k)); self.preset.blockSignals(False)

    def is_sim(self) -> bool:
        return self.preset.currentData() == "sim"

    def _on_preset(self, _i: int) -> None:
        key = self.preset.currentData(); _k, label, url, radio = next(p for p in PRESETS if p[0] == key); save_preset(key)
        if key == "custom":
            self.url.setFocus(); self.url.selectAll(); _status(self.video_status, "Custom: type the stream address, then press Start.", DIM); return
        self.url.setText(url)
        was = self.src.state in ("live", "connecting", "stalled")
        if was: self.stop_video()
        self.start_video()
        port = ""
        if radio == "sim": port = SIM_RADIO
        elif radio == "dongle":
            from .model import list_ports
            port = next((p for p in list_ports() if "CP2102" in p), "")
        msg = f"{label}: video {url}"
        if port and not self.win.model.connected:
            self.win.port_box.setCurrentText(port); msg += f"; radio port {port} (press Connect)"
        elif port:
            msg += "; the radio stays connected (disconnect first to switch it)"
        elif radio == "dongle":
            msg += "; no ELRS dongle (CP2102) found: plug it in and press Rescan"
        _status(self.video_status, msg, DIM)
        if hasattr(self.win, "say"): self.win.say(msg, "ok")                    # the video line is overwritten by the connection state: keep it in the main status line too

    def _target(self) -> str:
        return self.url.text().strip() or DEFAULT_URL

    def _tick(self) -> None:
        self._update_video(); self._cal_tick()

    def _track_box(self, box) -> None:
        vb = getattr(self.win, "vision_bar", None)
        if vb is None: return
        if not vb.btns["track"].isChecked(): vb.btns["track"].setChecked(True)
        vb.engine.track_box_request(box); _status(self.video_status, "TRACK: following the box you drew.", DIM)

    def start_video(self) -> None:
        self.src.set_source("network", self._target(), analog=False)
        self.src.start(); self._tick()

    MAP_SH = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "map.sh"))

    def _map_cmd(self, *args) -> str:
        """../../../map.sh: the mapping pipeline (o4_map_node + VO + RTAB-Map) fed by this GCS's O4 frames and FC telemetry."""
        import subprocess
        try:
            r = subprocess.run([self.MAP_SH, *args], capture_output=True, text=True, timeout=30)
            return (r.stdout.strip().splitlines() or [r.stderr.strip() or "done"])[-1]
        except Exception as e:                                                   # noqa: BLE001
            return f"map.sh failed: {e}"

    def start_map(self) -> None:
        """Starts the RTAB-Map pipeline on the live O4 video (the real camera is rectified; the simulator's already is the pinhole profile) and the RViz pane."""
        sim = self.is_sim() or "sim_radio" in (self.win.port_box.currentText() or "")
        _status(self.video_status, self._map_cmd("start", *([] if sim else ["--rectify"])), DIM)

    def stop_map(self) -> None:
        _status(self.video_status, self._map_cmd("stop"), DIM)

    def clear_map(self) -> None:
        msg = self._map_cmd("reset"); _status(self.video_status, msg, GOOD if "cleared" in msg else WARN)

    def showEvent(self, e) -> None:
        super().showEvent(e)                                              # (the map views read the window's one map client: win.map_client)

    def stop_video(self) -> None:
        self.src.stop(); self._tick()

    def snapshot(self) -> None:
        if self.src.latest() is None:
            _status(self.video_status, "No frame to save: start the video first.", WARN); return
        os.makedirs(SNAP_DIR, exist_ok=True)
        path = os.path.join(SNAP_DIR, time.strftime("%Y-%m-%d_%H%M%S") + "_snapshot.png")
        if self.src.snapshot_png(path):
            _status(self.video_status, f"Saved {path}", GOOD)
        else:
            _status(self.video_status, "Could not write the snapshot.", BAD)

    def _update_video(self) -> None:
        st = self.src.state; run = self.src.running
        self.btn_start.setEnabled(not run); self.btn_stop.setEnabled(run)
        self.btn_snap.setEnabled(self.src.latest() is not None)
        if st in ("live", "stalled"):
            d = self.src.health_snapshot()
            line = format_line(d)
            if self.probe.running and d["e2e_ms"] is None:
                line += "  ·  Latency test: no code read yet. Point the camera at the code window."
            _status(self.video_status, line, GOOD if d["state"] == "LIVE" else DIM if d["state"] == "STARTING" else WARN if d["state"] == "DEGRADED" else BAD)
        elif st == "connecting":
            _status(self.video_status, "Connecting\u2026", DIM)
        elif st.startswith("error") or st.startswith("OpenCV"):
            _status(self.video_status, st[:1].upper() + st[1:], BAD if st.startswith("error") else WARN)
        else:
            hint = "Enter the stream address, e.g. rtsp://192.168.50.1:554/video (turn on Enable RTSP server in CosmoStreamer first), then press Start."
            _status(self.video_status, hint, DIM)
        self.video_card.set_meta(f"{st.upper()}  {self.src.fps:.0f} FPS" if st in ("live", "stalled") else st.upper()[:24])
        self._view_tick(force=True)

    def _view_tick(self, force: bool = False) -> None:
        """Draw the newest frame as soon as it exists (30 Hz timer) and record how long after its arrival it was drawn."""
        st = self.src.state
        if st not in ("live", "stalled"):
            hf, _t = self.src.held_frame()
            if hf is not None and self.view.frame is not None:                  # a short drop: keep the last picture with a note, no blank / blink
                txt = f"VIDEO DROPPED \u00b7 {self.src.frame_age():.1f} s \u00b7 reconnecting"
                if txt != self.view.text: self.view.set_frame(self.view.frame, txt)
                return
            if self.view.frame is not None or force:
                self.view.set_frame(None, st)
            self._shown_t = None
            return
        f, t = self.src.latest_with_time()
        if f is None or (t == self._shown_t and not force):
            return
        compact = format_line(self.src.health_snapshot(), compact=True)
        vb = getattr(self.win, "vision_bar", None)
        if vb is not None: f = vb.engine.annotate(f)                       # DEPTH / DETECT / FACE / TRACK / VO overlays
        if self.cal is not None and self.cal.state == "collecting": f = self.cal.overlay(f)      # the board corners it found
        self.view.set_frame(f, compact)
        if t != self._shown_t:
            self._shown_t = t; self.src.note_shown(t)
            if self.probe.running:
                self.probe.feed(f, t)

    def toggle_latency_test(self) -> None:
        if self.lat_window is not None and self.lat_window.isVisible():
            self.lat_window.close(); self.probe.stop(); self.btn_lat.setText("Latency test"); return
        if self.lat_window is None:
            self.lat_window = LatencyCodeWindow()
        self.lat_window.show(); self.probe.start(); self.btn_lat.setText("Stop latency test")

    # ---- lens calibration (lens_calib.py): the real O4's K + distortion into gazebo/config/o4_camera.yaml, for the map's --rectify
    def toggle_calibration(self) -> None:
        import threading
        c = self.cal
        if c is not None and c.state == "collecting":                        # Finish: solve with the views taken (off the GUI thread)
            threading.Thread(target=c.finish, name="lens-solve", daemon=True).start(); self.btn_cal.setText("Calibrate lens"); return
        if self.is_sim():
            self._cal_say("Lens calibration is for the real O4: choose SOURCE  DJI O4 \u00b7 CosmoStreamer (direct)  and start its video first.", WARN); return
        if self.src.state != "live":
            self._cal_say("Start the O4 video first, then press Calibrate lens.", WARN); return
        from .lens_calib import LensCalibrator, config_model
        self.cal = LensCalibrator(self.src.latest_with_time, model=config_model())
        if self.cal_window is None: self.cal_window = CalibBoardWindow()
        self.cal_window.show(); self.btn_cal.setText("Finish calibration"); self._cal_tick()

    def _cal_say(self, text: str, color: str) -> None:
        self.cal_status.show(); _status(self.cal_status, text, color)

    def _cal_tick(self) -> None:
        c = self.cal
        if c is None: return
        from .lens_calib import TARGET_VIEWS
        if c.state == "collecting":
            board = f"board {c.pattern[0]}x{c.pattern[1]}" if c.pattern else "looking for the board"
            self._cal_say(f"LENS CALIBRATION ({c.model})  \u00b7  {board}  \u00b7  views {len(c.views)}/{TARGET_VIEWS}  \u00b7  picture covered {int(c.cover.sum())}/9  \u00b7  {c.hint}", DIM); return
        self._cal_say(c.hint, GOOD if c.state == "done" else DIM if c.state == "solving" else WARN)
        if c.state != "solving":
            self.btn_cal.setText("Calibrate lens")
            if self.cal_window is not None and self.cal_window.isVisible(): self.cal_window.close()

    def closeEvent(self, e) -> None:
        if self.cal is not None: self.cal.stop()
        if self.cal_window is not None: self.cal_window.close()
        self.probe.stop()
        if self.lat_window is not None:
            self.lat_window.close()
        super().closeEvent(e)
