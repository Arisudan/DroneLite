"""QGC-style left sidebar: search box + entries, each with a status dot AND a text state (readable without colour).

States: ok (blue filled dot, "OK"), caution (amber triangle, "CAUTION"), setup (hollow grey ring, "NEEDS SETUP").
`SectionStack` pairs the sidebar with a QStackedWidget of existing section pages.
"""
from __future__ import annotations

from PyQt6.QtCore import QPointF, QRectF, QSize, Qt, pyqtSignal
from PyQt6.QtGui import QPainter, QPen, QPolygonF
from PyQt6.QtWidgets import QHBoxLayout, QLineEdit, QScrollArea, QStackedWidget, QVBoxLayout, QWidget

from .. import theme

STATES = {"ok": "OK", "caution": "CAUTION", "setup": "NEEDS SETUP"}


class _Entry(QWidget):
    clicked = pyqtSignal(str)

    def __init__(self, key: str, title: str, state: str):
        super().__init__()
        self.key, self.title, self.state, self.active = key, title, state if state in STATES else "setup", False
        self.setFixedHeight(48); self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

    def sizeHint(self):
        return QSize(220, 48)

    def state_text(self) -> str:
        return STATES[self.state]

    def mousePressEvent(self, e):
        self.clicked.emit(self.key)

    def keyPressEvent(self, e):
        if e.key() in (Qt.Key.Key_Return, Qt.Key.Key_Enter, Qt.Key.Key_Space):
            self.clicked.emit(self.key)
        else:
            super().keyPressEvent(e)

    def paintEvent(self, _e):
        p = QPainter(self); p.setRenderHint(QPainter.RenderHint.Antialiasing)
        r = QRectF(self.rect())
        if self.active:
            p.fillRect(r, theme.qcolor(theme.HAIR)); p.fillRect(QRectF(0, 0, 2, r.height()), theme.qcolor(theme.ACC))
        cy = r.height() / 2
        col = {"ok": theme.ACC, "caution": theme.AMB, "setup": theme.FAINT}[self.state]
        c = QPointF(20, cy)
        if self.state == "ok":
            p.setPen(Qt.PenStyle.NoPen); p.setBrush(theme.qcolor(col)); p.drawEllipse(c, 5, 5)
        elif self.state == "caution":
            p.setPen(Qt.PenStyle.NoPen); p.setBrush(theme.qcolor(col))
            p.drawPolygon(QPolygonF([QPointF(20, cy - 6), QPointF(26, cy + 5), QPointF(14, cy + 5)]))
        else:
            p.setPen(QPen(theme.qcolor(col), 1.5)); p.setBrush(Qt.BrushStyle.NoBrush); p.drawEllipse(c, 5, 5)
        p.setPen(theme.qcolor(theme.TXT if self.active else theme.DIM)); p.setFont(theme.f_body(13))
        p.drawText(QRectF(38, 6, r.width() - 46, 20), Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignLeft, self.title)
        p.setPen(theme.qcolor(theme.AMB if self.state == "caution" else theme.FAINT)); p.setFont(theme.f_label(10))
        p.drawText(QRectF(38, 26, r.width() - 46, 16), Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignLeft, self.state_text())


class SidebarNav(QWidget):
    selected = pyqtSignal(str)

    def __init__(self, parent=None, width: int = 248):
        super().__init__(parent)
        self.setFixedWidth(width)
        self._entries: dict[str, _Entry] = {}
        self._current = ""
        lay = QVBoxLayout(self); lay.setContentsMargins(0, 0, 0, 0); lay.setSpacing(8)
        self.search = QLineEdit(); self.search.setPlaceholderText("Search"); self.search.setClearButtonEnabled(True)
        self.search.textChanged.connect(self.filter)
        lay.addWidget(self.search)
        self._box = QVBoxLayout(); self._box.setContentsMargins(0, 0, 0, 0); self._box.setSpacing(0)
        lay.addLayout(self._box); lay.addStretch(1)

    def add_entry(self, key: str, title: str, state: str = "setup") -> None:
        e = _Entry(key, title, state); e.clicked.connect(self.select)
        self._entries[key] = e; self._box.addWidget(e)
        if not self._current:
            self.select(key)

    def keys(self) -> list[str]:
        return list(self._entries)

    def current(self) -> str:
        return self._current

    def select(self, key: str) -> None:
        if key not in self._entries:
            return
        self._current = key
        for k, e in self._entries.items():
            e.active = k == key; e.update()
        self.selected.emit(key)

    def set_state(self, key: str, state: str) -> None:
        e = self._entries.get(key)
        if e and state in STATES and e.state != state:
            e.state = state; e.update()

    def state(self, key: str) -> str:
        return self._entries[key].state

    def state_text(self, key: str) -> str:
        return self._entries[key].state_text()

    def visible_keys(self) -> list[str]:
        return [k for k, e in self._entries.items() if not e.isHidden()]

    def filter(self, text: str) -> None:
        t = text.strip().lower()
        for e in self._entries.values():
            e.setVisible(not t or t in e.title.lower() or t in e.state_text().lower())


class SectionStack(QWidget):
    """Sidebar on the left, the selected section page on the right."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.nav = SidebarNav(); self.stack = QStackedWidget(); self._index: dict[str, int] = {}
        lay = QHBoxLayout(self); lay.setContentsMargins(24, 16, 24, 16); lay.setSpacing(24)
        lay.addWidget(self.nav); lay.addWidget(self.stack, 1)
        self.nav.selected.connect(lambda k: self.stack.setCurrentIndex(self._index[k]))

    def add_section(self, key: str, title: str, page: QWidget, state: str = "setup", scroll: bool = True) -> None:
        w = page
        if scroll:
            w = QScrollArea(); w.setWidgetResizable(True); w.setWidget(page)
        self._index[key] = self.stack.addWidget(w)
        self.nav.add_entry(key, title, state)
