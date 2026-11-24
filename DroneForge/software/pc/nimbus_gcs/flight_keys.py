"""Flight keys (2026-11-24, owner): hold A = ARM, hold T = TAKE OFF, press D = DISARM, press L = LAND (Esc = SAFE stays a QShortcut in main_window).

The keys call exactly what the FLY page buttons call (FlyPage.on_main / do_disarm / do_land -> the director), so every gate and refusal is the same: DISARM is refused in the
air by the director (use LAND, or SAFE), TAKE OFF only from ARMED, ARM only from the ground. A and T must be HELD for the same time as the on-screen hold button
(default_hold_ms, 1.0 s); letting go earlier cancels and says so. Ignored while typing (a text field, a number box or an editable list has the focus), with any modifier
(Ctrl / Alt / Shift / Meta), on key auto-repeat, and when the GCS window is not the active window."""
from __future__ import annotations

import time
from typing import Optional

from PyQt6.QtCore import QEvent, QObject, Qt, QTimer
from PyQt6.QtWidgets import QAbstractSpinBox, QApplication, QComboBox, QLineEdit, QPlainTextEdit, QTextEdit

from .components.hold_button import default_hold_ms

HOLD_KEYS = {Qt.Key.Key_A: "arm", Qt.Key.Key_T: "takeoff"}
TAP_KEYS = {Qt.Key.Key_D: "disarm", Qt.Key.Key_L: "land"}


def _typing(w) -> bool:
    if w is None: return False
    if isinstance(w, (QLineEdit, QTextEdit, QPlainTextEdit, QAbstractSpinBox)): return True
    return isinstance(w, QComboBox) and w.isEditable()


class FlightKeys(QObject):
    def __init__(self, win) -> None:
        super().__init__(win)
        self.win = win; self.held: Optional[int] = None; self.t0 = 0.0; self.fired = False
        self._timer = QTimer(self); self._timer.setSingleShot(True); self._timer.timeout.connect(self._hold_done)
        app = QApplication.instance()
        if app is not None: app.installEventFilter(self)

    # ---- the actions (the FLY page's own handlers)
    @property
    def fly(self):
        return getattr(self.win, "fly", None)

    def _phase(self) -> str:
        try: return self.fly.director.state().phase
        except Exception: return ""                                       # noqa: BLE001

    def _say(self, text: str, level: str = "warn") -> None:
        if hasattr(self.win, "say"): self.win.say(text, level)

    def do(self, action: str) -> None:
        f = self.fly
        if f is None: return
        if action == "arm":
            if self._phase() == "armed": self._say("Already armed: hold T to take off."); return
            f.on_main()
        elif action == "takeoff":
            if self._phase() != "armed": self._say("Take off needs the drone armed first: hold A to arm."); return
            f.on_main()
        elif action == "disarm":
            f.do_disarm()
        elif action == "land":
            f.do_land()

    # ---- keyboard
    def _ours(self, e) -> bool:
        if e.modifiers() & (Qt.KeyboardModifier.ControlModifier | Qt.KeyboardModifier.AltModifier | Qt.KeyboardModifier.ShiftModifier | Qt.KeyboardModifier.MetaModifier):
            return False
        if not self.win.isActiveWindow() and QApplication.activeWindow() is not None: return False
        return not _typing(QApplication.focusWidget())

    def eventFilter(self, obj, e) -> bool:
        t = e.type()
        if t not in (QEvent.Type.KeyPress, QEvent.Type.KeyRelease): return False
        k = e.key()
        if k not in HOLD_KEYS and k not in TAP_KEYS: return False
        if e.isAutoRepeat(): return k in HOLD_KEYS and self.held == k                 # swallow repeats of a held key
        if t == QEvent.Type.KeyPress:
            if not self._ours(e): return False
            if k in TAP_KEYS: self.do(TAP_KEYS[k]); return True
            self.held, self.t0, self.fired = k, time.monotonic(), False; ms = default_hold_ms()
            self._say(f"Hold {'A' if k == Qt.Key.Key_A else 'T'} for {ms / 1000:.1f} s to {'ARM' if k == Qt.Key.Key_A else 'TAKE OFF'}…", "ok")
            self._timer.start(ms); return True
        if self.held == k:                                                            # release
            self._timer.stop(); held, self.held = self.held, None
            if not self.fired: self._say(f"{'ARM' if held == Qt.Key.Key_A else 'TAKE OFF'} cancelled: the key was let go before {default_hold_ms() / 1000:.1f} s.")
            return True
        return False

    def _hold_done(self) -> None:
        if self.held is None: return
        self.fired = True; self.do(HOLD_KEYS[self.held])

    def cancel(self) -> None:
        self._timer.stop(); self.held = None
