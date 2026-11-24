import glob
import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
import unittest

import pytest
from PyQt6.QtCore import QEvent, QPoint, Qt
from PyQt6.QtGui import QKeyEvent
from PyQt6.QtTest import QTest
from PyQt6.QtWidgets import QApplication, QWidget

from nimbus_gcs import theme
from nimbus_gcs.components import ActionButton, Card, HoldButton, StatusChip, StepField, ToastHost, TopTabs, icons


@pytest.fixture(scope="module")
def app():
    a = QApplication.instance() or QApplication([]); theme.load_fonts(); return a


class Clk:
    t = 0.0
    def __call__(self):
        return self.t


def _hb(app, ms=1000):
    b = HoldButton("ARM", "PRESS AND HOLD 1.0 s", hold_ms=ms); b.resize(376, 96); b.show()
    b._clock = Clk(); fired = []; canc = []
    b.activated.connect(lambda: fired.append(1)); b.cancelled.connect(lambda: canc.append(1)); return b, fired, canc


def test_hold_fires_only_after_full_hold(app):
    b, fired, canc = _hb(app)
    QTest.mousePress(b, Qt.MouseButton.LeftButton)
    b._clock.t = 0.5; b._tick(); assert not fired and 0.45 < b.progress() < 0.55
    b._clock.t = 1.0; b._tick(); assert fired == [1] and b.progress() == 0
    QTest.mouseRelease(b, Qt.MouseButton.LeftButton); assert not canc


def test_early_release_cancels(app):
    b, fired, canc = _hb(app)
    QTest.mousePress(b, Qt.MouseButton.LeftButton); b._clock.t = 0.4
    QTest.mouseRelease(b, Qt.MouseButton.LeftButton)
    assert canc == [1] and not fired and b.progress() == 0


def test_disabled_never_fires(app):
    b, fired, canc = _hb(app); b.set_enabled_with_reason(False, "Waiting for link")
    QTest.mousePress(b, Qt.MouseButton.LeftButton); b._clock.t = 5; b._tick(); b.trigger()
    QTest.mouseRelease(b, Qt.MouseButton.LeftButton)
    assert not fired and not canc and b.reason() == "Waiting for link"
    b.set_enabled_with_reason(True); assert b.reason() == ""


def test_keyboard_hold_and_trigger_and_instant(app):
    b, fired, canc = _hb(app)
    QTest.keyPress(b, Qt.Key.Key_Space); b._clock.t = 1.2; b._tick(); QTest.keyRelease(b, Qt.Key.Key_Space)
    assert fired == [1]
    b.trigger(); assert len(fired) == 2
    b0, f0, _ = _hb(app, ms=0); QTest.mousePress(b0, Qt.MouseButton.LeftButton); assert f0 == [1]
    b.set_label("TAKE OFF"); b.set_hint("x"); assert b.label() == "TAKE OFF" and b.hint() == "x"


def test_env_hold_ms(app, monkeypatch):
    monkeypatch.setenv("DRONEFORGE_HOLD_MS", "0"); b = HoldButton(); fired = []; b.activated.connect(lambda: fired.append(1))
    QTest.mousePress(b, Qt.MouseButton.LeftButton); assert fired
    monkeypatch.delenv("DRONEFORGE_HOLD_MS"); assert HoldButton().hold_ms() == theme.HOLD_MS_DEFAULT


def test_action_button(app):
    a = ActionButton("DISARM", "unlock", "Available once armed"); a.resize(184, 72); got = []; a.clicked.connect(lambda: got.append(1))
    a.set_enabled_with_reason(False, "Available once armed"); QTest.mouseClick(a, Qt.MouseButton.LeftButton); assert not got and a.reason()
    a.set_enabled_with_reason(True); QTest.mouseClick(a, Qt.MouseButton.LeftButton); assert got == [1] and a.reason() == ""


def test_status_chip(app):
    c = StatusChip("BATT", "13.8 V", "battery"); w0 = c.sizeHint().width()
    c.set_value("13.2 V", "caution"); assert c.level() == "caution" and c.value() == "13.2 V"
    c.set_value("x", "bogus"); assert c.level() == "normal"
    c.set_value("0 %", "alarm"); assert c.level() == "alarm" and w0 > 0
    c.grab()


def test_toast(app):
    par = QWidget(); par.resize(800, 600); par.show(); h = ToastHost(par); got = []
    t = h.show_toast("Target 0.60 m", undo=lambda: got.append(1)); assert h.count() == 1 and t.undo_button
    QTest.mouseClick(t.undo_button, Qt.MouseButton.LeftButton); assert got == [1] and h.count() == 0
    h.show_toast("a", timeout_ms=30); QTest.qWait(150); assert h.count() == 0
    for i in range(5):
        h.show_toast(f"m{i}", timeout_ms=0)
    assert h.count() == 3 and h.messages() == ["m2", "m3", "m4"]
    g = h.geometry(); assert abs(g.center().x() - 400) <= 2 and g.bottom() < 600


def test_top_tabs(app):
    t = TopTabs(["FLY", "VIDEO", "SETUP"]); t.show(); seen = []; t.changed.connect(seen.append)
    QTest.keyClick(t, Qt.Key.Key_Right); QTest.keyClick(t, Qt.Key.Key_Right); QTest.keyClick(t, Qt.Key.Key_Right)
    assert seen == [1, 2] and t.current() == 2
    QTest.keyClick(t, Qt.Key.Key_Left); assert t.current() == 1
    t.select(0); assert seen[-1] == 0


def test_step_field(app):
    s = StepField(0.5, 0.05, 0.25, 1.5); seen = []; s.valueChanged.connect(seen.append); s.resize(328, 100); s.show()
    s.step_by(1); assert abs(s.value() - 0.55) < 1e-9 and seen
    s.set_value(9); assert s.value() == 1.5; s.set_value(-1); assert s.value() == 0.25
    s.step_by(-1); assert s.value() == 0.25
    s.set_value(0.5); QTest.mouseClick(s, Qt.MouseButton.LeftButton, pos=QPoint(s.width() - 20, 40)); assert abs(s.value() - 0.55) < 1e-9
    assert s.text() == "0.55" and "0.25" in s.range_text()


def test_card(app):
    c = Card("Target height", "0.25 - 1.50 m"); assert c.title() == "TARGET HEIGHT"; c.grab()


def test_icons_render(app):
    for n in icons.NAMES:
        im = icons.icon_pixmap(n, theme.TXT, 24).toImage(); assert any(im.pixelColor(x, y).alpha() for x in range(24) for y in range(24)), n
    assert icons.icon_pixmap("bell", theme.TXT, 24, dot=True)


def test_no_banned_widgets():
    d = os.path.dirname(icons.__file__)
    for f in glob.glob(d + "/*.py"):
        s = open(f).read()
        assert "QCheckBox" not in s and "QMessageBox" not in s, f


class _MP:                      # minimal monkeypatch for the unittest runner
    def setenv(self, k, v): os.environ[k] = v
    def delenv(self, k): os.environ.pop(k, None)


class ComponentTests(unittest.TestCase):
    """Runs the module-level tests under unittest too (df.py uses unittest discovery)."""


def _wrap(fn):
    def t(self):
        a = QApplication.instance() or QApplication([]); theme.load_fonts()
        n = fn.__code__.co_varnames[:fn.__code__.co_argcount]
        fn(*[{"app": a, "monkeypatch": _MP()}[x] for x in n])
    return t


for _n, _f in list(globals().items()):
    if _n.startswith("test_") and callable(_f):
        setattr(ComponentTests, _n, _wrap(_f))
