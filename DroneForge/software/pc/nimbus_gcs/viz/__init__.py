"""Data-visualisation widgets of the 'Mission control' look (timeline, height plot, flight view, scene, bottom strip, pre-flight list).

Shared QPainter helpers live here (hairlines aligned to device pixels, text with alignment, private icons drawn as QPainterPath).
All colours/fonts come from nimbus_gcs.theme.
"""
from __future__ import annotations

from PyQt6.QtCore import QPointF, QRectF, Qt
from PyQt6.QtGui import QColor, QFontMetricsF, QPainter, QPainterPath, QPen

from .. import theme as T


def col(token, alpha: int | None = None) -> QColor:
    c = QColor(token) if not isinstance(token, QColor) else QColor(token)
    if alpha is not None:
        c.setAlpha(alpha)
    return c


def text_w(s: str, font) -> float:
    return QFontMetricsF(font).horizontalAdvance(s) - font.letterSpacing()


def draw_text(p: QPainter, x: float, y: float, s: str, font, color, align: str = "l") -> float:
    """Draw `s` with baseline y; align l/r/c about x. Returns the text width."""
    p.setFont(font); p.setPen(col(color))
    w = text_w(s, font)
    if align == "r":
        x -= w
    elif align == "c":
        x -= w / 2
    p.drawText(QPointF(x, y), s)
    return w


def hline(p: QPainter, x0: float, x1: float, y: float, color=T.HAIR) -> None:
    p.fillRect(QRectF(round(x0), round(y), round(x1) - round(x0), 1), col(color))


def vline(p: QPainter, x: float, y0: float, y1: float, color=T.HAIR) -> None:
    p.fillRect(QRectF(round(x), round(y0), 1, round(y1) - round(y0)), col(color))


def _pen(color, w=1.5) -> QPen:
    q = QPen(col(color), w); q.setCapStyle(Qt.PenCapStyle.RoundCap); q.setJoinStyle(Qt.PenJoinStyle.RoundJoin); return q


def icon_check(p: QPainter, cx: float, cy: float, color, w: float = 1.5) -> None:
    path = QPainterPath(); path.moveTo(cx - 4, cy + .5); path.lineTo(cx - 1, cy + 3.5); path.lineTo(cx + 4.5, cy - 3)
    p.setPen(_pen(color, w)); p.setBrush(Qt.BrushStyle.NoBrush); p.drawPath(path)


def icon_cross(p: QPainter, cx: float, cy: float, color, w: float = 1.6) -> None:
    path = QPainterPath(); path.moveTo(cx - 3.5, cy - 3.5); path.lineTo(cx + 3.5, cy + 3.5); path.moveTo(cx + 3.5, cy - 3.5); path.lineTo(cx - 3.5, cy + 3.5)
    p.setPen(_pen(color, w)); p.setBrush(Qt.BrushStyle.NoBrush); p.drawPath(path)


def icon_tri(p: QPainter, cx: float, cy: float, color, w: float = 1.5) -> None:
    path = QPainterPath(); path.moveTo(cx, cy - 6); path.lineTo(cx + 6.5, cy + 5); path.lineTo(cx - 6.5, cy + 5); path.closeSubpath()
    p.setPen(_pen(color, w)); p.setBrush(Qt.BrushStyle.NoBrush); p.drawPath(path)
    p.drawLine(QPointF(cx, cy - 2), QPointF(cx, cy + 1.2))
    p.setPen(Qt.PenStyle.NoPen); p.setBrush(col(color)); p.drawEllipse(QPointF(cx, cy + 3.2), .9, .9)


def icon_arrow(p: QPainter, cx: float, cy: float, color, up: bool = True, w: float = 1.6) -> None:
    s = 1 if up else -1
    path = QPainterPath(); path.moveTo(cx, cy + 6 * s); path.lineTo(cx, cy - 6 * s)
    path.moveTo(cx - 4, cy - 2 * s); path.lineTo(cx, cy - 6.5 * s); path.lineTo(cx + 4, cy - 2 * s)
    p.setPen(_pen(color, w)); p.setBrush(Qt.BrushStyle.NoBrush); p.drawPath(path)


def fmt_mmss(sec: float) -> str:
    sec = max(0, int(round(sec)))
    return f"{sec // 60}:{sec % 60:02d}"


def new_painter(w) -> QPainter:
    p = QPainter(w)
    p.setRenderHints(QPainter.RenderHint.Antialiasing | QPainter.RenderHint.TextAntialiasing | QPainter.RenderHint.SmoothPixmapTransform)
    return p
