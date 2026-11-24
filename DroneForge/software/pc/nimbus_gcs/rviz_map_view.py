"""The map inside the GCS, drawn exactly as RViz draws it (DRONE_GCS approach): RTAB-Map's /map and the wall skeleton /map_thin from the map stream
(vision/map_stream_node.py, TCP 5765), with RViz's own palettes, background, grid and TF axes, and RViz's TopDownOrtho mouse controls.

Why not the embedded rviz2 window: on this desktop (COSMIC, Wayland; X11 programs through XWayland) a foreign X11 window re-parented into the GCS is not composed
inside it: it shows as its own window. So the 2D view is native here; "Open RViz" starts the full rviz2 (same layout, rtabmap_drone.rviz) for 3D and all its panels.

Colours (copied from DRONE_GCS scripts/gcs/core/map_render.py, which reads them out of librviz_default_plugins.so: makeMapPalette / makeCostmapPalette):
  /map       "map" scheme, alpha 0.7: free (0) white .. occupied (100) black, unknown (-1) teal-grey 112,137,134
  /map_thin  "costmap" scheme, alpha 1.0: free transparent, 1..98 blue->red, 99 cyan, 100 magenta, unknown teal-grey
  background 48,48,48; Grid display: 1 m cells, 10 x 10 plane at the origin, 160,160,160 at alpha 0.7
Mouse (RViz TopDownOrtho): wheel = zoom, left drag = rotate, middle drag or shift + left drag = move, right drag = zoom, double click = the layout's start view."""
from __future__ import annotations

import math
import time

import numpy as np
from PyQt6.QtCore import QPointF, QRectF, Qt, QTimer
from PyQt6.QtGui import QColor, QFont, QImage, QPainter, QPen, QTransform
from PyQt6.QtWidgets import QHBoxLayout, QPushButton, QWidget

from . import theme as T

# ---- RViz palettes (DRONE_GCS core/map_render.py)
_UNKNOWN = [112, 137, 134, 255]; _ILL_POS = [0, 255, 0, 255]


def _ill_neg(idx):
    return [255, (255 * (idx - 128)) // 126, 0, 255]


RAW_LUT = np.zeros((256, 4), np.uint8); THIN_LUT = np.zeros((256, 4), np.uint8)
for _v in range(-128, 128):
    _i = _v & 0xFF
    if 0 <= _v <= 100: _s = 255 - (255 * _v) // 100; RAW_LUT[_i] = [_s, _s, _s, 255]
    elif 101 <= _v <= 127: RAW_LUT[_i] = _ILL_POS
    elif _v == -1: RAW_LUT[_i] = _UNKNOWN
    else: RAW_LUT[_i] = _ill_neg(_i)
    if _v == 0: THIN_LUT[_i] = [0, 0, 0, 0]
    elif 1 <= _v <= 98: _r = (255 * _v) // 100; THIN_LUT[_i] = [_r, 0, 255 - _r, 255]
    elif _v == 99: THIN_LUT[_i] = [0, 255, 255, 255]
    elif _v == 100: THIN_LUT[_i] = [255, 0, 255, 255]
    elif 101 <= _v <= 127: THIN_LUT[_i] = _ILL_POS
    elif _v == -1: THIN_LUT[_i] = _UNKNOWN
    else: THIN_LUT[_i] = _ill_neg(_i)

BG = QColor(48, 48, 48); GRID = QColor(160, 160, 160, 178); MAP_ALPHA = 0.7; THIN_ALPHA = 1.0
START = dict(scale=167.4273681640625, angle=-0.049999989569187164, cx=0.0, cy=0.0)        # rtabmap_drone.rviz "Current View"


def grid_image(item, lut):
    g = item["grid"]; h, w = g.shape
    rgba = np.ascontiguousarray(lut[g.view(np.uint8)])                                     # row 0 = the grid's south edge: drawn at origin_y in a y-up world transform
    return QImage(rgba.data, w, h, 4 * w, QImage.Format.Format_RGBA8888).copy()


class RvizMapView(QWidget):
    def __init__(self, win) -> None:
        super().__init__()
        self.win = win; self.setMinimumSize(240, 200); self.setMouseTracking(False); self.setFocusPolicy(Qt.FocusPolicy.ClickFocus)
        self.v = dict(START); self._drag = None; self._imgs = {}; self._stamp = {}
        # toolbar (2026-11-24): RST restarts the whole mapping pipeline; Open RViz = the full rviz2 window
        self.bar = QWidget(self); bl = QHBoxLayout(self.bar); bl.setContentsMargins(0, 0, 0, 0); bl.setSpacing(6)
        self.btn_rst = QPushButton("\u27f3  RST", self.bar); self.btn_rviz = QPushButton("Open RViz", self.bar)
        self.btn_rst.setToolTip("Restart the map: stops and restarts the mapping pipeline (depth, visual odometry, RTAB-Map, map stream) and reconnects. Use it when the map lags or is stuck.")
        self.btn_rviz.setToolTip("The full rviz2 (same layout): 3D orbit view, Displays and Views panels")
        for b in (self.btn_rst, self.btn_rviz):
            b.setFixedHeight(28); b.setCursor(Qt.CursorShape.PointingHandCursor); b.setFont(T.f_body(12))
            b.setStyleSheet(f"QPushButton{{background:{T.PANEL};color:{T.TXT};border:1px solid {T.HAIR2};border-radius:6px;padding:0px 12px;}}"
                            f"QPushButton:hover{{border-color:{T.ACC};}} QPushButton:pressed{{background:{T.HAIR};}}"
                            f"QPushButton:disabled{{color:{T.DISABLED};border:1px dashed {T.HAIR2};}}")
            bl.addWidget(b)
        self.btn_rviz.clicked.connect(self._open_rviz); self.btn_rst.clicked.connect(self.restart_map)
        self._rst_state = ""; self._rst_t = 0.0
        self._t = QTimer(self); self._t.timeout.connect(self._poll); self._t.start(100)

    # ---- data
    @property
    def client(self):
        return getattr(self.win, "map_client", None)

    def _poll(self) -> None:
        if self._rst_state and self._rst_state != "running" and not self.btn_rst.isEnabled():
            self.btn_rst.setEnabled(True); self.update()
        c = self.client
        if c is None or not self.isVisible(): return
        changed = False
        for key, lut in (("raw", RAW_LUT), ("thin", THIN_LUT)):
            item = getattr(c, key, None)
            if item is not None and self._stamp.get(key) != item["t"]:
                self._imgs[key] = (grid_image(item, lut), item); self._stamp[key] = item["t"]; changed = True
            elif item is None and key in self._imgs:
                del self._imgs[key]; changed = True
        pose = None if c.pose is None else tuple(round(float(v), 3) for v in c.pose)
        if changed or pose != getattr(self, "_pose", None): self._pose = pose; self.update()      # repaint only on a change: the GUI thread also draws the video

    def _open_rviz(self) -> None:
        from .rviz_embed import launch_rviz_window
        self.btn_rviz.setToolTip(launch_rviz_window())

    # ---- view transform (RViz TopDownOrtho: x right, y up at angle 0, rotated by the view angle)
    def _world(self) -> QTransform:
        t = QTransform(); t.translate(self.width() / 2.0, self.height() / 2.0); t.scale(self.v["scale"], -self.v["scale"])
        t.rotate(math.degrees(self.v["angle"])); t.translate(-self.v["cx"], -self.v["cy"]); return t

    def resizeEvent(self, e) -> None:
        self.bar.adjustSize(); self.bar.move(self.width() - self.bar.width() - 10, 10); super().resizeEvent(e)

    # ---- RST: restart the mapping pipeline (in a worker thread: map.sh takes seconds)
    MAP_SH = __import__("os").path.abspath(__import__("os").path.join(__import__("os").path.dirname(__file__), "..", "..", "..", "map.sh"))

    def _is_sim(self) -> bool:
        vp = getattr(self.win, "__dict__", {}).get("video_page")
        if vp is not None and hasattr(vp, "is_sim"): return vp.is_sim()
        try:
            from .video_page import load_preset
            return load_preset() == "sim" or "sim_radio" in (self.win.port_box.currentText() or "")
        except Exception:                                                    # noqa: BLE001
            return True

    def restart_map(self) -> None:
        if self._rst_state == "running": return
        import subprocess, threading
        rect = [] if self._is_sim() else ["--rectify"]
        self._rst_state = "running"; self._rst_t = time.time(); self.btn_rst.setEnabled(False); self._imgs.clear(); self._stamp.clear()
        c = self.client
        if c is not None:
            try: c.clear()
            except Exception: pass                                           # noqa: BLE001

        def work():
            try:
                subprocess.run([self.MAP_SH, "stop"], capture_output=True, timeout=30)
                r = subprocess.run([self.MAP_SH, "start", *rect], capture_output=True, text=True, timeout=60)
                self._rst_state = "done" if r.returncode == 0 else "failed: " + ((r.stderr or r.stdout).strip().splitlines() or ["map.sh"])[-1]
            except Exception as e:                                            # noqa: BLE001
                self._rst_state = f"failed: {e}"
        threading.Thread(target=work, name="map-restart", daemon=True).start(); self.update()

    def paintEvent(self, _e) -> None:
        p = QPainter(self); p.fillRect(self.rect(), BG)
        p.setRenderHint(QPainter.RenderHint.SmoothPixmapTransform, False)
        p.save(); p.setTransform(self._world())
        for key, alpha in (("raw", MAP_ALPHA), ("thin", THIN_ALPHA)):
            if key in self._imgs:
                img, it = self._imgs[key]; h, w = it["grid"].shape
                p.setOpacity(alpha); p.drawImage(QRectF(it["ox"], it["oy"], w * it["res"], h * it["res"]), img)
        p.setOpacity(1.0)
        pen = QPen(GRID); pen.setCosmetic(True); pen.setWidthF(1.0); p.setPen(pen)                  # Grid display: 10 x 10 cells of 1 m at the origin
        for k in range(-5, 6):
            p.drawLine(QPointF(k, -5.0), QPointF(k, 5.0)); p.drawLine(QPointF(-5.0, k), QPointF(5.0, k))
        self._axes(p, 0.0, 0.0, 0.0)                                                                 # TF: the map frame
        c = self.client
        if c is not None and c.pose is not None: self._axes(p, *c.pose)                               # TF: base_link (the drone) from RTAB-Map's map -> base_link
        p.restore()
        p.setFont(QFont(T.f_body(10).family(), 9)); p.setPen(QColor(220, 220, 220))
        if self._rst_state == "running":
            st = f"restarting the map pipeline\u2026 {time.time() - self._rst_t:.0f} s"
        elif self._rst_state.startswith("failed") and time.time() - self._rst_t < 20:
            st = "map restart " + self._rst_state
        elif not self._imgs:
            st = "waiting for the map stream (map.sh / sim.sh start)" if (c is None or c.state != "connected") else "waiting for RTAB-Map's first map: take off"
        else:
            st = ""
        if st:
            p.setPen(QColor(T.AMB) if self._rst_state.startswith("failed") else QColor(220, 220, 220))
            p.drawText(QRectF(12, 14, self.width() - self.bar.width() - 36, 20), Qt.AlignmentFlag.AlignLeft, st)
        p.setPen(QColor(220, 220, 220))
        p.drawText(QRectF(10, self.height() - 22, self.width() - 20, 18), Qt.AlignmentFlag.AlignLeft,
                   "wheel zoom · left drag rotate · middle / shift+left move · right drag zoom · double click reset")

    def _axes(self, p: QPainter, x: float, y: float, yaw: float, length: float = 0.3) -> None:
        for col, ang in ((QColor(255, 0, 0), yaw), (QColor(0, 255, 0), yaw + math.pi / 2)):            # RViz TF axes: x red, y green
            pen = QPen(col); pen.setCosmetic(True); pen.setWidthF(3.0); p.setPen(pen)
            p.drawLine(QPointF(x, y), QPointF(x + length * math.cos(ang), y + length * math.sin(ang)))

    # ---- mouse: RViz TopDownOrtho
    def wheelEvent(self, e) -> None:
        self.v["scale"] = float(np.clip(self.v["scale"] * (1.1 ** (e.angleDelta().y() / 120.0)), 2.0, 5000.0)); self.update()

    def mousePressEvent(self, e) -> None:
        b = e.button(); shift = bool(e.modifiers() & Qt.KeyboardModifier.ShiftModifier)
        mode = "move" if (b == Qt.MouseButton.MiddleButton or (b == Qt.MouseButton.LeftButton and shift)) else "rotate" if b == Qt.MouseButton.LeftButton else "zoom" if b == Qt.MouseButton.RightButton else None
        self._drag = (mode, e.position(), dict(self.v)) if mode else None

    def mouseMoveEvent(self, e) -> None:
        if not self._drag: return
        mode, p0, v0 = self._drag; dx, dy = e.position().x() - p0.x(), e.position().y() - p0.y()
        if mode == "rotate":
            self.v["angle"] = v0["angle"] + dx * 0.005
        elif mode == "zoom":
            self.v["scale"] = float(np.clip(v0["scale"] * (1.0 - dy * 0.005), 2.0, 5000.0))
        else:                                                                                          # move: screen delta -> world delta (undo scale and rotation)
            s, a = v0["scale"], v0["angle"]; wx, wy = dx / s, -dy / s
            self.v["cx"] = v0["cx"] - (wx * math.cos(-a) - wy * math.sin(-a)); self.v["cy"] = v0["cy"] - (wx * math.sin(-a) + wy * math.cos(-a))
        self.update()

    def mouseReleaseEvent(self, e) -> None:
        self._drag = None

    def mouseDoubleClickEvent(self, e) -> None:
        self.v = dict(START); self.update()
