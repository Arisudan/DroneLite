"""Vector icons (QPainterPath, 1.5-px round strokes) in the concept-2 style. Each `draw_<name>(painter, rect_or_center, color, size=20)` draws
centred on a point (or the centre of a rect); designs are authored on a ~20-unit box and scaled by size/20. `icon_pixmap(name, color, px)` renders one."""
from __future__ import annotations

from PyQt6.QtCore import QPointF, QRectF, Qt
from PyQt6.QtGui import QColor, QPainter, QPainterPath, QPen, QPixmap


def _pen(p, col, w=1.5):
    q = QPen(QColor(col), w); q.setCapStyle(Qt.PenCapStyle.RoundCap); q.setJoinStyle(Qt.PenJoinStyle.RoundJoin)
    p.setPen(q); p.setBrush(Qt.BrushStyle.NoBrush)


def _dot(p, col, x, y, r):
    p.setPen(Qt.PenStyle.NoPen); p.setBrush(QColor(col)); p.drawEllipse(QPointF(x, y), r, r); p.setBrush(Qt.BrushStyle.NoBrush)


def _poly(pts, close=False):
    path = QPainterPath(); path.moveTo(*pts[0])
    for pt in pts[1:]:
        path.lineTo(*pt)
    if close:
        path.closeSubpath()
    return path


def _logo(p, c, **k):
    _pen(p, c); p.drawPath(_poly([(0, -9), (8, 7), (0, 3), (-8, 7)], True))


def _bell(p, c, dot=False, **k):
    _pen(p, c)
    b = QPainterPath(); b.moveTo(-6, 4); b.cubicTo(-4, 2, -4.5, -1, -4.5, -3); b.cubicTo(-4.5, -7, 4.5, -7, 4.5, -3)
    b.cubicTo(4.5, -1, 4, 2, 6, 4); b.closeSubpath(); p.drawPath(b)
    a = QPainterPath(); a.moveTo(-2, 7); a.cubicTo(-1.5, 9, 1.5, 9, 2, 7); p.drawPath(a)
    if dot:
        _dot(p, c, 5, -6, 2.2)


def _check(p, c, **k):
    _pen(p, c, 1.6); p.drawPath(_poly([(-4, .5), (-1, 3.5), (4.5, -3)]))


def _caution(p, c, **k):
    _pen(p, c); p.drawPath(_poly([(0, -6.5), (6.8, 5), (-6.8, 5)], True))
    p.drawLine(QPointF(0, -2), QPointF(0, 1.2)); _dot(p, c, 0, 3.2, .9)


def _lock(p, c, **k):
    _pen(p, c, 1.5); p.drawRoundedRect(QRectF(-5.5, -1, 11, 8.5), 2, 2)
    sh = QPainterPath(); sh.moveTo(-3.2, -1); sh.lineTo(-3.2, -3.5); sh.cubicTo(-3.2, -8.5, 3.2, -8.5, 3.2, -3.5); sh.lineTo(3.2, -1); p.drawPath(sh)


def _unlock(p, c, **k):
    _pen(p, c, 1.5); p.drawRoundedRect(QRectF(-5.5, -1, 11, 8.5), 2, 2)
    sh = QPainterPath(); sh.moveTo(-3.2, -1); sh.lineTo(-3.2, -4); sh.cubicTo(-3.2, -9, 3.2, -9, 3.2, -5.5); p.drawPath(sh)


def _download(p, c, **k):
    _pen(p, c, 1.6); p.drawLine(QPointF(0, -8), QPointF(0, 3)); p.drawPath(_poly([(-5, -2), (0, 4), (5, -2)]))
    p.drawLine(QPointF(-8, 8), QPointF(8, 8))


def _drone(p, c, **k):
    _pen(p, c, 1.6)
    for dx, dy in ((-6, -6), (6, -6), (-6, 6), (6, 6)):
        p.drawEllipse(QPointF(dx, dy), 3.2, 3.2)
    p.drawLine(QPointF(-3.5, -3.5), QPointF(3.5, 3.5)); p.drawLine(QPointF(3.5, -3.5), QPointF(-3.5, 3.5))


def _gear(p, c, **k):
    import math
    _pen(p, c, 1.4); p.drawEllipse(QPointF(0, 0), 2.8, 2.8)
    for i in range(8):
        a = math.radians(i * 45); p.drawLine(QPointF(5.2 * math.cos(a), 5.2 * math.sin(a)), QPointF(7.6 * math.cos(a), 7.6 * math.sin(a)))
    p.drawEllipse(QPointF(0, 0), 5.4, 5.4)


def _video(p, c, **k):
    _pen(p, c, 1.5); p.drawRoundedRect(QRectF(-8, -5, 11, 10), 2, 2); p.drawPath(_poly([(3, -1.5), (8, -4.5), (8, 4.5), (3, 1.5)], True))


def _sliders(p, c, **k):
    _pen(p, c, 1.5)
    for y, x in ((-5, -2), (0, 3), (5, -4)):
        p.drawLine(QPointF(-8, y), QPointF(8, y))
    for y, x in ((-5, -2), (0, 3), (5, -4)):
        p.setBrush(QColor("#000000")); p.setBrush(Qt.BrushStyle.NoBrush); p.drawEllipse(QPointF(x, y), 2, 2)


def _chart(p, c, **k):
    _pen(p, c, 1.5); p.drawPath(_poly([(-8, -7), (-8, 7), (8, 7)])); p.drawPath(_poly([(-5, 3), (-1, -1), (2, 1.5), (7, -5)]))


def _search(p, c, **k):
    _pen(p, c, 1.6); p.drawEllipse(QPointF(-1.5, -1.5), 5.5, 5.5); p.drawLine(QPointF(2.5, 2.5), QPointF(7.5, 7.5))


def _plus(p, c, **k):
    _pen(p, c, 1.6); p.drawLine(QPointF(-7, 0), QPointF(7, 0)); p.drawLine(QPointF(0, -7), QPointF(0, 7))


def _minus(p, c, **k):
    _pen(p, c, 1.6); p.drawLine(QPointF(-7, 0), QPointF(7, 0))


def _chevron(p, c, **k):
    _pen(p, c, 1.6); p.drawPath(_poly([(-3, -5), (3, 0), (-3, 5)]))


def _close(p, c, **k):
    _pen(p, c, 1.6); p.drawLine(QPointF(-5, -5), QPointF(5, 5)); p.drawLine(QPointF(5, -5), QPointF(-5, 5))


def _wifi(p, c, **k):
    _pen(p, c, 1.4)
    for r in (3, 6, 9):
        a = QPainterPath(); rect = QRectF(-r, 4 - r, 2 * r, 2 * r); a.arcMoveTo(rect, 125); a.arcTo(rect, 125, -70); p.drawPath(a)
    _dot(p, c, 0, 4, 1.1)


def _battery(p, c, **k):
    _pen(p, c, 1.4); p.drawRoundedRect(QRectF(-7, -4, 12, 8), 1.8, 1.8); p.drawLine(QPointF(7, -1.5), QPointF(7, 1.5))


def _mode(p, c, **k):
    _pen(p, c, 1.4); path = QPainterPath(); path.moveTo(-8, 0)
    path.cubicTo(-5, -8, -3, -8, 0, 0); path.cubicTo(3, 8, 5, 8, 8, 0); p.drawPath(path)


_ICONS = {"logo": _logo, "bell": _bell, "check": _check, "caution": _caution, "lock": _lock, "unlock": _unlock, "download": _download,
          "drone": _drone, "gear": _gear, "video": _video, "sliders": _sliders, "chart": _chart, "search": _search, "plus": _plus,
          "minus": _minus, "chevron": _chevron, "close": _close, "wifi": _wifi, "battery": _battery, "mode": _mode}
NAMES = tuple(_ICONS)


def draw_icon(name: str, painter: QPainter, where, color, size: float = 20, **kw) -> None:
    cx = where.center() if isinstance(where, QRectF) or hasattr(where, "center") else where
    painter.save(); painter.setRenderHint(QPainter.RenderHint.Antialiasing)
    painter.translate(QPointF(cx)); s = size / 20.0; painter.scale(s, s)
    # strokes keep their nominal 1.5 px at size 20; scale proportionally for other sizes
    _ICONS[name](painter, color, **kw)
    painter.restore()


def _make(name):
    def f(painter, where, color, size=20, **kw):
        draw_icon(name, painter, where, color, size, **kw)
    f.__name__ = "draw_" + name
    return f


for _n in _ICONS:
    globals()["draw_" + _n] = _make(_n)


def icon_pixmap(name: str, color, px: int = 20, dpr: float = 1.0, **kw) -> QPixmap:
    pm = QPixmap(int(px * dpr), int(px * dpr)); pm.setDevicePixelRatio(dpr); pm.fill(Qt.GlobalColor.transparent)
    p = QPainter(pm); draw_icon(name, p, QPointF(px / 2, px / 2), color, px, **kw); p.end()
    return pm
