"""PreflightList: header, check rows (ok / caution / blocking) and an advisory footnote."""
from __future__ import annotations

from PyQt6.QtCore import QRectF, QSize, Qt, pyqtSignal
from PyQt6.QtGui import QFontMetrics
from PyQt6.QtWidgets import QSizePolicy, QWidget

from .. import theme as T
from . import col, draw_text, hline, icon_check, icon_cross, icon_tri, new_painter

ROW_H = 38             # (rows stretch 24..40)
HEAD_H = 24
FOOT = "Amber rows are warnings only: they never block arming."


class PreflightList(QWidget):
    row_clicked = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        T.load_fonts()
        self._rows: list = []; self._hover = -1; self._row_h = 24; self._foot = False; self._compact = False
        self.setMouseTracking(True)
        self._fit()

    def set_checks(self, rows) -> None:
        self._rows = [tuple(r) for r in rows]; self._fit(); self.update()

    def ready_count(self) -> int:
        return sum(1 for r in self._rows if r[3] != "blocking")

    def set_compact(self, on: bool) -> None:
        """Small windows: 24 px rows and no footnote line."""
        self._compact = on; self._fit(); self.update()

    def min_height_for(self, row_h: int, foot: bool) -> int:
        return HEAD_H + 8 + row_h * len(self._rows) + (32 if foot else 4)

    def _fit(self) -> None:
        """Minimum = 24 px rows, no footnote; the rows stretch (24..40 px) with the height the column gives, the footnote shows only if room is left."""
        self.setMinimumHeight(self.min_height_for(24, False))
        self.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Expanding)
        n = max(1, len(self._rows)); h = self.height()
        self._row_h = max(24, min(ROW_H + 2, (h - HEAD_H - 12) // n)) if h > 0 else 24
        self._foot = h - HEAD_H - 8 - self._row_h * n >= 30

    def resizeEvent(self, e):
        self._fit(); super().resizeEvent(e)

    def sizeHint(self) -> QSize:
        return QSize(376, self.minimumHeight())

    def _row_at(self, y: float) -> int:
        i = int((y - (HEAD_H + 8)) // self._row_h)
        return i if 0 <= i < len(self._rows) and y >= HEAD_H + 8 else -1

    def mouseMoveEvent(self, e):
        i = self._row_at(e.position().y())
        if i != self._hover:
            self._hover = i; self.update()
            self.setToolTip(f"{self._rows[i][1]}: {self._rows[i][2]}" if i >= 0 else "")

    def leaveEvent(self, _):
        self._hover = -1; self.update()

    def mouseReleaseEvent(self, e):
        i = self._row_at(e.position().y())
        if i >= 0 and e.button() == Qt.MouseButton.LeftButton:
            self.row_clicked.emit(self._rows[i][0])

    def paintEvent(self, _):
        p = new_painter(self); W = self.width()
        draw_text(p, 0, 12, "PRE-FLIGHT", T.f_label(11), T.DIM)
        draw_text(p, W, 14, f"{self.ready_count()} of {len(self._rows)} ready", T.f_body(13), T.TXT, "r")
        y = HEAD_H + 8
        hline(p, 0, W, y - 8, T.HAIR)
        for i, (rid, label, state, level) in enumerate(self._rows):
            cy = y + self._row_h / 2
            if level == "caution":
                p.fillRect(QRectF(0, y + 1, W, self._row_h - 1), col(T.AMB, 14))
            elif level == "blocking":
                p.fillRect(QRectF(0, y + 1, W, self._row_h - 1), col(T.RED, 16))
            elif i == self._hover:
                p.fillRect(QRectF(0, y + 1, W, self._row_h - 1), col("#FFFFFF", 6))
            if level == "caution":
                icon_tri(p, 12, cy, T.AMB); sc, tf = T.AMB, T.f_body(12); tf.setWeight(tf.Weight.DemiBold); lc = T.TXT
            elif level == "blocking":
                icon_cross(p, 12, cy, T.RED); sc, tf = T.RED, T.f_body(12); tf.setWeight(tf.Weight.Bold); lc = T.TXT
            else:
                icon_check(p, 12, cy, T.DIM); sc, tf = T.FAINT, T.f_label(10); lc = col(T.TXT, 220)
            lf = T.f_body(13 if self._row_h < 34 else 14); draw_text(p, 32, cy + 5, label, lf, lc)
            txt = state.upper() if level == "ok" else state
            room = W - 8 - (32 + QFontMetrics(lf).horizontalAdvance(label) + 14)               # never run into the label: elide the reading to what is left
            txt = QFontMetrics(tf).elidedText(txt, Qt.TextElideMode.ElideRight, max(40, int(room)))
            draw_text(p, W - 8, cy + 5, txt, tf, sc, "r")
            y += self._row_h
            hline(p, 0, W, y, T.HAIR)
        if self._foot:
            draw_text(p, 0, y + 22, FOOT, T.f_body(11), T.FAINT)
        p.end()
