"""SceneView: the main view. Dark horizon scene (or live video) with HUD corner readouts, crosshair and an optional PiP."""
from __future__ import annotations

import math
import random

from PyQt6.QtCore import QEvent, QPointF, QRectF, QSize, Qt, pyqtSignal
from PyQt6.QtGui import (QImage, QLinearGradient, QPainter, QPainterPath, QPen, QPixmap, QRadialGradient)
from PyQt6.QtWidgets import QWidget

from .. import theme as T
from . import col, draw_text, icon_arrow, new_painter, text_w

LABEL_C = "#C5CBD4"


def render_scene(w: int, h: int, dpr: float = 1.0) -> QPixmap:
    """The dark horizon scene (sky, hills, tree line, ground, perspective grid, landing pad, haze, vignette)."""
    pm = QPixmap(int(w * dpr), int(h * dpr)); pm.setDevicePixelRatio(dpr); pm.fill(col(T.BG))
    p = QPainter(pm); p.setRenderHints(QPainter.RenderHint.Antialiasing)
    k = w / 1472.0; hor = h * 0.50
    sky = QLinearGradient(0, 0, 0, hor); sky.setColorAt(0, col("#0F1520")); sky.setColorAt(.55, col("#1B2736")); sky.setColorAt(1, col("#3A4658"))
    p.fillRect(QRectF(0, 0, w, hor), sky)
    g = QRadialGradient(QPointF(w * .66, hor), w * .42); g.setColorAt(0, col("#E8D6BE", 70)); g.setColorAt(1, col("#E8D6BE", 0))
    p.fillRect(QRectF(0, 0, w, hor), g)
    hills = QPainterPath(); hills.moveTo(0, hor)
    for i in range(0, int(w) + 8, 8):
        hills.lineTo(i, hor - 22 * k - 18 * k * math.sin(i / (210 * k) + .6) - 9 * k * math.sin(i / (71 * k)))
    hills.lineTo(w, hor); hills.closeSubpath()
    hg = QLinearGradient(0, hor - 50, 0, hor); hg.setColorAt(0, col("#202B38")); hg.setColorAt(1, col("#2A3543")); p.fillPath(hills, hg)
    rnd = random.Random(7); tree = QPainterPath(); tree.moveTo(0, hor + 2); xx = 0.0
    while xx < w:
        th = (10 + rnd.random() * 14 + (8 if 300 * k < xx < 520 * k or 1000 * k < xx < 1180 * k else 0)) * max(.7, k)
        tree.lineTo(xx, hor - th * .6); tree.lineTo(xx + 5 * k, hor - th); tree.lineTo(xx + 10 * k, hor - th * .55); xx += max(6.0, 10 * k)
    tree.lineTo(w, hor + 2); tree.closeSubpath(); p.fillPath(tree, col("#141B24"))
    gg = QLinearGradient(0, hor, 0, h); gg.setColorAt(0, col("#262D2F")); gg.setColorAt(.35, col("#1B2123")); gg.setColorAt(1, col("#0D1012"))
    p.fillRect(QRectF(0, hor, w, h - hor), gg)
    vx = w * .5; p.setPen(QPen(col("#FFFFFF", 9), 1))
    for i in range(-14, 15):
        p.drawLine(QPointF(vx + i * 18 * k, hor), QPointF(vx + i * 150 * k, h))
    for j in range(1, 9):
        yy = hor + ((j / 9.0) ** 2) * (h - hor); p.drawLine(QPointF(0, yy), QPointF(w, yy))
    cx, cy = vx + 24 * k, hor + (h - hor) * .5
    p.save(); p.translate(cx, cy); p.scale(1, .26)
    pg = QRadialGradient(QPointF(0, 0), 190 * k); pg.setColorAt(0, col("#5C6468", 150)); pg.setColorAt(1, col("#3C4246", 110))
    p.setBrush(pg); p.setPen(QPen(col("#D2D8DE", 70), 3)); p.drawEllipse(QPointF(0, 0), 188 * k, 188 * k)
    p.setPen(QPen(col("#D2D8DE", 55), 2)); p.drawEllipse(QPointF(0, 0), 140 * k, 140 * k); p.restore()
    hz = QLinearGradient(0, hor - 30, 0, hor + 50); hz.setColorAt(0, col("#788CA5", 0)); hz.setColorAt(.5, col("#788CA5", 34)); hz.setColorAt(1, col("#788CA5", 0))
    p.fillRect(QRectF(0, hor - 30, w, 80), hz)
    vg = QRadialGradient(QPointF(w / 2, h / 2), w * .62); vg.setColorAt(.55, col("#000000", 0)); vg.setColorAt(1, col("#000000", 150))
    p.fillRect(QRectF(0, 0, w, h), vg)
    p.end(); return pm


class SceneView(QWidget):
    pip_clicked = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        T.load_fonts()
        self._frame: QImage | None = None; self._cache: QPixmap | None = None; self._cache_key = None
        self._alt = 0.0; self._hdg = 0.0; self._vs = 0.0
        self._pip: QWidget | None = None
        self._tag = ""
        self.setMinimumSize(320, 130)

    def set_frame(self, img) -> None:
        self._frame = img if (img is not None and not img.isNull()) else None; self.update()

    def set_video_tag(self, text: str) -> None:
        """Small corner text over the helipad fallback ('VIDEO OFF' / 'VIDEO CONNECTING' / 'VIDEO STALLED'); '' hides it."""
        if text != self._tag:
            self._tag = text; self.update()

    def video_tag(self) -> str:
        return self._tag

    def has_frame(self) -> bool:
        return self._frame is not None

    def set_values(self, alt_m: float, hdg_deg: float, vs_mps: float) -> None:
        self._alt, self._hdg, self._vs = float(alt_m), float(hdg_deg) % 360, float(vs_mps); self.update()

    def set_pip_widget(self, w) -> None:
        if self._pip is not None:
            self._pip.removeEventFilter(self); self._pip.setParent(None)
        self._pip = w
        if w is not None:
            w.setParent(self); w.installEventFilter(self); w.setCursor(Qt.CursorShape.PointingHandCursor); w.show(); self._place_pip()

    def pip_rect(self):
        if self._pip is None:
            return None
        pw = int(max(240, min(360, self.width() * .21))); ph = int(pw * .67)
        return (16, self.height() - ph - 16, pw, ph)

    def _place_pip(self) -> None:
        r = self.pip_rect()
        if r:
            self._pip.setGeometry(*r); self._pip.raise_()

    def resizeEvent(self, e):
        self._place_pip(); super().resizeEvent(e)

    def eventFilter(self, o, e):
        if o is self._pip and e.type() == QEvent.Type.MouseButtonRelease:
            self.pip_clicked.emit(); return True
        return super().eventFilter(o, e)

    def sizeHint(self) -> QSize:
        return QSize(1472, 488)

    def paintEvent(self, _):
        p = new_painter(self); W, H = self.width(), self.height()
        dpr = self.devicePixelRatioF(); clip = QPainterPath(); clip.addRoundedRect(QRectF(0, 0, W, H), 8, 8); p.setClipPath(clip)
        if self._frame is not None:
            p.fillRect(QRectF(0, 0, W, H), col("#000000"))
            fs = self._frame.size(); s = min(W / fs.width(), H / fs.height()); fw, fh = fs.width() * s, fs.height() * s
            p.setRenderHint(QPainter.RenderHint.SmoothPixmapTransform, False)       # fast scaling for live video
            p.drawImage(QRectF((W - fw) / 2, (H - fh) / 2, fw, fh), self._frame)
        else:
            key = (W, H, dpr)
            if key != self._cache_key:
                self._cache = render_scene(W, H, dpr); self._cache_key = key
            p.drawPixmap(0, 0, self._cache)
        for (y0, y1, a0, a1) in ((0, 160, 150, 0), (H - 140, H, 0, 150)):
            sc = QLinearGradient(0, y0, 0, y1); sc.setColorAt(0, col("#000000", a0)); sc.setColorAt(1, col("#000000", a1)); p.fillRect(QRectF(0, y0, W, y1 - y0), sc)
        if self._tag and self._frame is None:
            tf = T.f_label(11); tw = text_w(self._tag, tf)
            p.fillRect(QRectF(24, 84, tw + 16, 22), col("#000000", 150))
            draw_text(p, 32, 99, self._tag, tf, LABEL_C)
        # crosshair ticks
        p.setPen(QPen(col("#FFFFFF", 120), 1)); mx, my = round(W / 2) + .5, round(H / 2) + .5
        for a, b in ((-24, -10), (10, 24)):
            p.drawLine(QPointF(mx + a, my), QPointF(mx + b, my)); p.drawLine(QPointF(mx, my + a), QPointF(mx, my + b))
        # HUD
        lf, nf, uf = T.f_label(11), T.f_l(40), T.f_body(13)
        draw_text(p, 24, 28, "ALT", lf, LABEL_C); w = draw_text(p, 24, 68, f"{self._alt:.2f}", nf, T.TXT); draw_text(p, 24 + w + 6, 68, "m", uf, LABEL_C)
        draw_text(p, W - 24, 28, "HDG", lf, LABEL_C, "r"); draw_text(p, W - 24, 68, f"{self._hdg:03.0f}°", nf, T.TXT, "r")
        vs = f"{self._vs:+.2f}"; vw = text_w(vs, nf); uw = text_w("m/s", uf); x0 = W - 24 - (22 + vw + 6 + uw)
        draw_text(p, x0, H - 67, "V/S", lf, LABEL_C)
        icon_arrow(p, x0 + 7, H - 37, T.TXT, up=self._vs >= 0)
        draw_text(p, x0 + 22, H - 23, vs, nf, T.TXT); draw_text(p, x0 + 22 + vw + 6, H - 23, "m/s", uf, LABEL_C)
        p.setClipping(False); p.setPen(QPen(col("#FFFFFF", 22), 1)); p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawPath(self._rr(W, H)); p.end()

    @staticmethod
    def _rr(W, H):
        q = QPainterPath(); q.addRoundedRect(QRectF(.5, .5, W - 1, H - 1), 8, 8); return q
