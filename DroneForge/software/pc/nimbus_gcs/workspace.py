"""Workspace = a sub-tab bar (segmented buttons, shown only when there is more than one page) + a stack of lazily built pages.
Also the tiny persisted UI state (last workspace / sub-tab) in ~/.config/droneforge/gcs_ui.json (env DRONEFORGE_UI_STATE redirects it)."""
from __future__ import annotations

import json
import os
from typing import Callable, List, Optional

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtWidgets import (QButtonGroup, QFrame, QHBoxLayout, QLabel, QPushButton, QScrollArea, QStackedWidget, QVBoxLayout, QWidget)


def state_path() -> str:
    return os.environ.get("DRONEFORGE_UI_STATE") or os.path.expanduser("~/.config/droneforge/gcs_ui.json")


def load_state() -> dict:
    try:
        with open(state_path(), encoding="utf-8") as f:
            d = json.load(f)
        return d if isinstance(d, dict) else {}
    except Exception:
        return {}


def save_state(d: dict) -> None:
    try:
        p = state_path(); os.makedirs(os.path.dirname(p) or ".", exist_ok=True)
        with open(p, "w", encoding="utf-8") as f:
            json.dump(d, f)
    except Exception:
        pass


def placeholder(name: str) -> QWidget:
    lab = QLabel(f"{name} not available yet"); lab.setObjectName("dim"); lab.setAlignment(Qt.AlignmentFlag.AlignCenter); return lab


def lazy_import(module: str, cls: str, win, label: str) -> QWidget:
    """Build `module.cls(win)` from the package, or a placeholder when the module is missing (parallel development)."""
    try:
        import importlib
        mod = importlib.import_module(f"nimbus_gcs.{module}")
        return getattr(mod, cls)(win)
    except (ImportError, AttributeError):
        return placeholder(label)


class Workspace(QWidget):
    changed = pyqtSignal(int)

    def __init__(self, name: str) -> None:
        super().__init__()
        self.name = name
        self._titles: List[str] = []; self._factories: List[Callable[[], QWidget]] = []; self._slots: List[Optional[QWidget]] = []; self._pages: List[Optional[QWidget]] = []
        l = QVBoxLayout(self); l.setContentsMargins(0, 0, 0, 0); l.setSpacing(0)
        self.bar = QFrame(); self.bar.setObjectName("subBar"); self.bar.setFixedHeight(40)
        self._pad: List[bool] = []; self._scroll: List[bool] = []
        self.bar_l = QHBoxLayout(self.bar); self.bar_l.setContentsMargins(24, 0, 24, 0); self.bar_l.setSpacing(4)
        self.bar_l.addStretch(1)
        self.group = QButtonGroup(self); self.group.setExclusive(True); self.group.idClicked.connect(self.select)
        self.buttons: List[QPushButton] = []
        self.bar.setVisible(False)
        self.stack = QStackedWidget()
        l.addWidget(self.bar); l.addWidget(self.stack, 1)
        self._cur = -1

    # --- API -------------------------------------------------------------------------------------
    def add_page(self, title: str, factory: Callable[[], QWidget], scroll: bool = True, pad: bool = True, fit: bool = False) -> int:
        """fit=True: the page is never wrapped in a scroll area (it must lay itself out elastically, as Fly does). pad=False: the page runs edge to edge (Fly); otherwise it gets the 24 px gutters of the look."""
        i = len(self._titles); self._pad.append(pad); self._scroll.append(not fit)
        self._titles.append(title); self._factories.append(factory); self._pages.append(None)
        holder = QWidget(); QVBoxLayout(holder).setContentsMargins(*((24, 8, 24, 16) if pad else (0, 0, 0, 0)))
        self._slots.append(holder); self.stack.addWidget(holder)
        b = QPushButton(title.upper()); b.setObjectName("seg"); b.setCheckable(True); b.setCursor(Qt.CursorShape.PointingHandCursor)
        self.group.addButton(b, i); self.buttons.append(b); self.bar_l.insertWidget(self.bar_l.count() - 1, b)
        self.bar.setVisible(len(self._titles) > 1)
        if self._cur < 0:
            self._cur = 0; b.setChecked(True)
        return i

    def count(self) -> int:
        return len(self._titles)

    def titles(self) -> List[str]:
        return list(self._titles)

    def index_of(self, title: str) -> int:
        t = title.lower()
        for i, x in enumerate(self._titles):
            if x.lower() == t:
                return i
        return -1

    def current_index(self) -> int:
        return max(self._cur, 0)

    def current_title(self) -> str:
        return self._titles[self._cur] if 0 <= self._cur < len(self._titles) else ""

    def build(self, i: int) -> QWidget:
        """Build page i if needed and return it."""
        if self._pages[i] is None:
            scroll_ok = self._scroll[i]
            w = self._factories[i]()
            if not scroll_ok:
                self._pages[i] = w; self._slots[i].layout().addWidget(w); return w
            sa = QScrollArea(); sa.setWidgetResizable(True); sa.setFrameShape(QFrame.Shape.NoFrame); sa.setWidget(w)
            sa.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
            self._pages[i] = w; self._slots[i].layout().addWidget(sa)
            self._scrolls = getattr(self, "_scrolls", {}); self._scrolls[i] = sa
        return self._pages[i]

    def page(self, i: int) -> Optional[QWidget]:
        """The page if already built, else None."""
        return self._pages[i] if 0 <= i < len(self._pages) else None

    def scroll_area(self, i: int):
        return getattr(self, "_scrolls", {}).get(i)

    def built_pages(self) -> List[QWidget]:
        return [p for p in self._pages if p is not None]

    def select(self, i: int) -> None:
        if not 0 <= i < len(self._titles):
            return
        self.build(i)
        self._cur = i
        self.buttons[i].setChecked(True); self.stack.setCurrentIndex(i)
        self.changed.emit(i)

    def preset(self, title: str) -> None:
        """Choose the sub-tab without building it (built on first show): used when restoring the saved state."""
        i = self.index_of(title)
        if i >= 0:
            self._cur = i; self.buttons[i].setChecked(True); self.stack.setCurrentIndex(i)

    def select_title(self, title: str) -> bool:
        i = self.index_of(title)
        if i < 0:
            return False
        self.select(i); return True

    def cycle(self, step: int) -> None:
        n = self.count()
        if n > 1:
            self.select((self.current_index() + step) % n)

    def showEvent(self, e) -> None:
        super().showEvent(e)
        if self._titles:
            self.build(self.current_index())
