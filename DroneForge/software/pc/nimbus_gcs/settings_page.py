"""Settings page: one grouped form built entirely from nimbus_core.settings.SCHEMA.

Left list of groups, fields on the right. Edits commit immediately through
director.apply_settings; results/refusals are shown inline (no dialogs, no tick boxes).
"""
from __future__ import annotations

import dataclasses

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (QComboBox, QDoubleSpinBox, QGridLayout, QHBoxLayout, QLabel,
                             QPushButton, QStackedWidget, QVBoxLayout, QWidget)

from . import theme
from .components import Card
from .components.sidebar_nav import SidebarNav

from nimbus_core import settings as S
from nimbus_core.battery_cal import calibrate_battery_scale

_GREEN, _RED, _GREY = theme.ACC, theme.RED, theme.DIM


def _decimals(step) -> int:
    if not step:
        return 2
    s = f"{step:.6f}".rstrip("0")
    return max(0, len(s.split(".")[1])) if "." in s else 0


class SettingsPage(QWidget):
    def __init__(self, win):
        super().__init__()
        self.win = win
        self._widgets: dict[str, QWidget] = {}
        self._fields: dict[str, dict] = {}
        self._notes: dict[str, QLabel] = {}
        self._group_keys: list[list[str]] = []
        self._loading = False
        self.setMinimumWidth(760); self.setObjectName("body"); self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)

        root = QVBoxLayout(self)
        root.setContentsMargins(24, 16, 24, 16)

        body = QHBoxLayout(); body.setSpacing(24)
        self.nav = SidebarNav()
        self.stack = QStackedWidget()
        body.addWidget(self.nav); body.addWidget(self.stack, 1)
        root.addLayout(body, 1)

        self.status = QLabel(""); self.status.setWordWrap(True); self.status.setMinimumHeight(24)
        root.addWidget(self.status)

        for g in S.SCHEMA:
            self.nav.add_entry(g["group"], g["group"], "ok")
            self.stack.addWidget(self._build_group(g))
        self._names = [g["group"] for g in S.SCHEMA]
        self.nav.selected.connect(lambda k: self.stack.setCurrentIndex(self._names.index(k)))
        self.refresh()

    # ---- building --------------------------------------------------------------------------
    def _build_group(self, g) -> QWidget:
        page = QWidget(); lay = QVBoxLayout(page); lay.setContentsMargins(0, 0, 0, 0)
        card = Card(g["group"]); cl = card.body
        grid = QGridLayout(); grid.setHorizontalSpacing(16); grid.setVerticalSpacing(10)
        grid.setColumnStretch(1, 1)
        keys = []
        row = 0
        for f in g["fields"]:
            key = f["key"]; self._fields[key] = f
            lab = QLabel(f["label"]); lab.setObjectName("dim"); lab.setToolTip(f.get("tooltip") or "")
            grid.addWidget(lab, row, 0)
            w = self._make_widget(f)
            w.setToolTip(f.get("tooltip") or "")
            self._widgets[key] = w
            grid.addWidget(w, row, 1)
            if not f.get("readonly"):
                keys.append(key)
            row += 1
            if f.get("tooltip"):                                          # the explanation is shown, not hidden in a tooltip
                hint = QLabel(f["tooltip"]); hint.setWordWrap(True); hint.setObjectName("faint"); grid.addWidget(hint, row, 1); row += 1
            if key == "battery_scale":
                b1 = QPushButton("Calibrate: pack is fully charged"); b2 = QPushButton("Reset factor to 1.00"); b1.setObjectName("primary")
                b1.clicked.connect(lambda _=False: self._calibrate()); b2.clicked.connect(lambda _=False: self._apply(battery_scale=1.0))
                self.btn_cal, self.btn_cal_reset = b1, b2
                hb = QHBoxLayout(); hb.addWidget(b1); hb.addWidget(b2); hb.addStretch(1)
                grid.addLayout(hb, row, 1); row += 1
            note = QLabel(""); note.setWordWrap(True); note.setStyleSheet(f"color:{theme.AMB};")
            note.hide(); self._notes[key] = note
            grid.addWidget(note, row, 1)
            row += 1
        cl.addLayout(grid)
        self._group_keys.append(keys)
        btn = QPushButton("Reset group to defaults"); btn.setObjectName("ghost")
        btn.clicked.connect(lambda _=False, i=len(self._group_keys) - 1: self._reset_group(i))
        h = QHBoxLayout(); h.addStretch(1); h.addWidget(btn); cl.addLayout(h)
        lay.addWidget(card); lay.addStretch(1)
        return page

    def _make_widget(self, f) -> QWidget:
        key = f["key"]
        if f.get("readonly"):
            v = f.get("value")
            return QLabel(f"{v:g} {f['unit']}".strip() if isinstance(v, (int, float)) else str(v))
        if f["kind"] == "choice":
            exp_vals = set(f.get("experimental_values") or [])
            cb = QComboBox()
            for val, text in f["choices"]:
                if (val in exp_vals or f.get("experimental")) and "experimental" not in text.lower():
                    text += " (experimental)"
                cb.addItem(text, val)
            cb.activated.connect(lambda _=0, k=key: self._commit(k))
            return cb
        sp = QDoubleSpinBox()
        sp.setDecimals(_decimals(f["step"])); sp.setRange(f["min"], f["max"])
        sp.setSingleStep(f["step"] or 0.1)
        if f["unit"]:
            sp.setSuffix(" " + f["unit"])
        sp.setKeyboardTracking(False)
        sp.editingFinished.connect(lambda k=key: self._commit(k))
        return sp

    # ---- values ----------------------------------------------------------------------------
    def _current(self):
        return self.win.model.core.autonomy.settings

    def _set_widget(self, key, value):
        w = self._widgets[key]
        if isinstance(w, QDoubleSpinBox):
            w.blockSignals(True); w.setValue(float(value)); w.blockSignals(False)
        elif isinstance(w, QComboBox):
            i = w.findData(value)
            if i >= 0:
                w.blockSignals(True); w.setCurrentIndex(i); w.blockSignals(False)
            self._update_note(key)

    def _update_note(self, key):
        f, w = self._fields[key], self._widgets[key]
        if not isinstance(w, QComboBox):
            return
        exp = set(f.get("experimental_values") or [])
        on = f.get("experimental") or w.currentData() in exp
        self._notes[key].setText("Experimental: not flight-proven. Use only with the owner's go-ahead." if on else "")
        self._notes[key].setVisible(bool(on))

    def refresh(self):
        cur = self._current()
        for key, w in self._widgets.items():
            if not self._fields[key].get("readonly"):
                self._set_widget(key, getattr(cur, key))

    def _value(self, key):
        w = self._widgets[key]
        return w.currentData() if isinstance(w, QComboBox) else round(w.value(), 6)

    def _say(self, text, ok):
        self.status.setText(text)
        self.status.setStyleSheet(f"color:{_GREEN if ok else _RED}; font-weight:600;")
        self.status.setProperty("ok", ok)

    def _apply(self, **values) -> bool:
        cur = self._current()
        res = self.win.model.core.director.apply_settings(dataclasses.replace(cur, **values))
        ok = bool(res.ok)
        if ok:
            self._say("Saved: " + ", ".join(self._fields[k]["label"] for k in values)
                      + f" ({res.message})", True)
        else:
            self._say(str(res.message), False)
        self.refresh()          # on refusal this reverts widgets to the value in force
        return ok

    def _commit(self, key):
        self._update_note(key)
        cur = getattr(self._current(), key)
        v = self._value(key)
        if v == cur:
            return
        self._apply(**{key: v})

    def _calibrate(self):
        m = self.win.model
        if getattr(m, "is_sim", False):
            return self._say("Cannot calibrate: this is the simulator, not a real battery reading.", False)
        raw = m.raw_vbat() if hasattr(m, "raw_vbat") else None
        cells = getattr(getattr(getattr(m.core, "safety", None), "cfg", None), "cells", 4)
        if raw is None:
            return self._say("Cannot calibrate: no battery telemetry (connect with the pack plugged in).", False)
        if raw < 3.0 * cells:
            return self._say(f"Cannot calibrate: the FC reads {raw:.1f} V, too low for a charged {cells}S pack "
                             f"(needs at least {3.0 * cells:.1f} V). Charge the pack first.", False)
        f = round(calibrate_battery_scale(raw, cells), 2)
        if self._apply(battery_scale=f):
            self._say(f"Factor {f:.2f}: {raw:.1f} V reads as {raw * f:.1f} V ({raw * f / cells:.2f} V/cell)", True)

    def _reset_group(self, idx):
        d = S.FlightSettings()
        keys = self._group_keys[idx]
        if keys:
            self._apply(**{k: getattr(d, k) for k in keys})

    def showEvent(self, e):
        self.refresh()
        super().showEvent(e)
