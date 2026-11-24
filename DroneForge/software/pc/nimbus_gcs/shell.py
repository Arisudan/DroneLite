"""Shell pieces of the 'Mission control' look: the top bar widgets (logo, notification bell + list popup, source popup) and the Ctrl+K command palette.
No flight logic here and no dialogs other than the plain palette list (it asks nothing: Enter runs the highlighted entry)."""
from __future__ import annotations

import time
from typing import Callable, List, Tuple

from PyQt6.QtCore import QPointF, QRectF, Qt, pyqtSignal
from PyQt6.QtGui import QPainter, QPen
from PyQt6.QtWidgets import QDialog, QFrame, QLabel, QLineEdit, QListWidget, QListWidgetItem, QVBoxLayout, QWidget

from . import theme as T
from .components import icons


class Logo(QWidget):
    def __init__(self) -> None:
        super().__init__(); self.setFixedSize(28, 32)

    def paintEvent(self, e) -> None:
        p = QPainter(self); icons.draw_icon("logo", p, QPointF(14, 16), T.qcolor(T.TXT), 22)


class BellButton(QWidget):
    """Notification bell with the unread count as a number (not colour alone)."""
    clicked = pyqtSignal()

    def __init__(self) -> None:
        super().__init__(); self._n = 0
        self.setFixedHeight(32); self.setFixedWidth(32); self.setCursor(Qt.CursorShape.PointingHandCursor); self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self.setToolTip("Notifications")

    def count(self) -> int:
        return self._n

    def set_count(self, n: int) -> None:
        if n != self._n:
            self._n = n; self.setFixedWidth(32 if n == 0 else 56); self.updateGeometry(); self.update()

    def mouseReleaseEvent(self, e) -> None:
        if e.button() == Qt.MouseButton.LeftButton and self.rect().contains(e.position().toPoint()):
            self.clicked.emit()

    def keyPressEvent(self, e) -> None:
        if e.key() in (Qt.Key.Key_Return, Qt.Key.Key_Enter, Qt.Key.Key_Space):
            self.clicked.emit()
        else:
            super().keyPressEvent(e)

    def paintEvent(self, e) -> None:
        p = QPainter(self); p.setRenderHint(QPainter.RenderHint.Antialiasing); p.setRenderHint(QPainter.RenderHint.TextAntialiasing)
        p.setPen(QPen(T.qcolor(T.HAIR2), 1)); p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawRoundedRect(QRectF(self.rect()).adjusted(.5, .5, -.5, -.5), 16 if self._n == 0 else 6, 16 if self._n == 0 else 6)
        icons.draw_icon("bell", p, QPointF(16, 16), T.qcolor(T.TXT), 18)
        if self._n:
            p.setFont(T.f_mono(12)); p.setPen(T.qcolor(T.ACC))
            p.drawText(QRectF(30, 0, 22, 32), Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignLeft, str(self._n) if self._n < 100 else "99+")


class _PopupFrame(QFrame):
    def __init__(self, parent: QWidget, width: int) -> None:
        super().__init__(parent, Qt.WindowType.Popup)
        self.setObjectName("card"); self.setFixedWidth(width)
        self.setStyleSheet(f"QFrame#card {{ background:{T.PANEL}; border:1px solid {T.HAIR2}; border-radius:{T.RADIUS['m']}px; }}")
        self.col = QVBoxLayout(self); self.col.setContentsMargins(16, 12, 16, 12); self.col.setSpacing(8)

    def show_under(self, anchor: QWidget) -> None:
        self.adjustSize()
        g = anchor.mapToGlobal(QPointF(0, anchor.height() + 6).toPoint())
        x = max(8, g.x() + anchor.width() - self.width())
        self.move(x, g.y()); self.show()


class NotificationPopup(_PopupFrame):
    """Plain list of the latest notices, newest first. Opening it marks them read."""

    def __init__(self, parent: QWidget) -> None:
        super().__init__(parent, 420)
        h = QLabel("NOTIFICATIONS"); h.setObjectName("label"); h.setFont(T.f_label()); self.col.addWidget(h)
        self.list = QListWidget(); self.list.setFixedHeight(300); self.list.setFocusPolicy(Qt.FocusPolicy.NoFocus); self.col.addWidget(self.list)

    def set_notices(self, notices) -> None:
        self.list.clear()
        if not notices:
            it = QListWidgetItem("No notifications"); it.setFlags(Qt.ItemFlag.NoItemFlags); self.list.addItem(it); return
        for n in reversed(list(notices)):
            tag = {"caution": "CAUTION  ", "alarm": "ALARM  "}.get(n.level, "")
            self.list.addItem(f"{time.strftime('%H:%M:%S', time.localtime(n.t))}   {tag}{n.text}")


class CommandPalette(QDialog):
    """Ctrl+K: type to filter, Enter or double-click runs the entry. Entries are (label, callback); nothing asks for confirmation."""

    def __init__(self, parent: QWidget) -> None:
        super().__init__(parent)
        self.setWindowTitle("Command palette"); self.setWindowFlag(Qt.WindowType.WindowContextHelpButtonHint, False); self.resize(520, 420)
        self._entries: List[Tuple[str, Callable[[], None]]] = []
        l = QVBoxLayout(self); l.setContentsMargins(16, 16, 16, 16); l.setSpacing(8)
        self.edit = QLineEdit(); self.edit.setPlaceholderText("Type to jump to a workspace or run an action"); l.addWidget(self.edit)
        self.list = QListWidget(); self.list.setFocusPolicy(Qt.FocusPolicy.NoFocus); l.addWidget(self.list, 1)
        self.edit.textChanged.connect(self._filter); self.edit.returnPressed.connect(self.run_current)
        self.list.itemDoubleClicked.connect(lambda _i: self.run_current())
        self.edit.installEventFilter(self)

    def set_entries(self, entries) -> None:
        self._entries = list(entries); self._filter(self.edit.text())

    def labels(self) -> List[str]:
        return [self.list.item(i).text() for i in range(self.list.count())]

    def _filter(self, text: str) -> None:
        q = text.strip().lower(); self.list.clear()
        for label, _fn in self._entries:
            if all(w in label.lower() for w in q.split()):
                self.list.addItem(label)
        if self.list.count():
            self.list.setCurrentRow(0)

    def open_palette(self) -> None:
        self.edit.clear(); self._filter(""); self.show(); self.raise_(); self.activateWindow(); self.edit.setFocus()

    def run_current(self) -> bool:
        it = self.list.currentItem()
        if it is None:
            return False
        fn = next((f for lab, f in self._entries if lab == it.text()), None)
        self.hide()
        if fn:
            fn()
        return fn is not None

    def eventFilter(self, o, e) -> bool:
        if o is self.edit and e.type() == e.Type.KeyPress and e.key() in (Qt.Key.Key_Down, Qt.Key.Key_Up):
            r = self.list.currentRow() + (1 if e.key() == Qt.Key.Key_Down else -1)
            self.list.setCurrentRow(max(0, min(self.list.count() - 1, r))); return True
        return super().eventFilter(o, e)
