"""BottomStrip (FLY page): the flight readouts under the height chart, evenly spread with hairline dividers: HEIGHT | TARGET | ERROR (height - target) | RANGE.
2026-11-24 (owner): LQ and BATT are in the top bar (LINK chip, battery gauge); the strip lives under the left column so the right column (pre-flight list) runs to the
bottom of the page. A rounded panel, label above value, tabular figures; amber + a triangle only for a caution (e.g. the height error)."""
from __future__ import annotations

from PyQt6.QtCore import QRectF, QSize
from PyQt6.QtGui import QPen
from PyQt6.QtWidgets import QWidget

from .. import theme as T
from . import col, draw_text, icon_tri, new_painter

CELLS = [("HEIGHT", "m"), ("TARGET", "m"), ("ERROR", "m"), ("RANGE", "m")]
STRIP_H, STRIP_H_COMPACT = 56, 46


class BottomStrip(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        T.load_fonts()
        self._v = {n: "—" for n, _ in CELLS}; self._caution: set[str] = set()
        self._compact = False; self.setFixedHeight(STRIP_H)

    def set_values(self, d: dict) -> None:
        for k, v in d.items():
            k = str(k).upper()
            if k in self._v:
                self._v[k] = str(v)
        self.update()

    def set_caution(self, name: str, on: bool = True) -> None:
        (self._caution.add if on else self._caution.discard)(name.upper()); self.update()

    def set_compact(self, on: bool) -> None:
        self._compact = on; self.setFixedHeight(STRIP_H_COMPACT if on else STRIP_H); self.updateGeometry(); self.update()

    def sizeHint(self) -> QSize:
        return QSize(640, self.height())

    def paintEvent(self, _):
        p = new_painter(self); W, H = self.width(), self.height()
        r = QRectF(0.5, 0.5, W - 1, H - 1)
        p.setPen(QPen(col(T.HAIR), 1)); p.setBrush(col(T.PANEL)); p.drawRoundedRect(r, T.RADIUS["m"], T.RADIUS["m"])
        cw = W / len(CELLS); small = self._compact
        for i, (name, unit) in enumerate(CELLS):
            x0 = i * cw; v = self._v[name]; caut = name in self._caution
            if i:
                p.setPen(QPen(col(T.HAIR), 1)); p.drawLine(int(x0), 10, int(x0), int(H - 10))
            x = x0 + 20
            draw_text(p, x, (16 if small else 19), name, T.f_label(9 if small else 10), T.DIM if caut else T.FAINT)
            vc = T.AMB if caut else (T.DIM if v == "—" else T.TXT)
            xe = x + draw_text(p, x, H - (10 if small else 12), v, T.f_mono(16 if small else 19), vc)
            if unit and v != "—":
                xe += 5 + draw_text(p, xe + 5, H - (10 if small else 12), unit, T.f_body(11), T.FAINT)
            if caut:
                icon_tri(p, xe + 14, H - (16 if small else 19), T.AMB)
        p.end()
