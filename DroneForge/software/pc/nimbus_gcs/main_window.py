"""Main window shell: header (status chips + SAFE), connection strip, a left navigation rail and a page stack, with an events log under the pages.
The Fly page lives in fly_page.py and the Settings page in settings_page.py; the other pages are assembled in _build_pages(). No dialogs anywhere:
refusals and errors go to inline, colour-coded status lines."""

import time

from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QColor, QFont, QKeySequence, QPalette, QShortcut
from PyQt6.QtWidgets import (QAbstractSpinBox, QApplication, QComboBox, QFrame, QHBoxLayout, QLabel, QLineEdit, QMainWindow, QPlainTextEdit,
                             QPushButton, QSizePolicy, QStackedWidget, QTextEdit, QVBoxLayout, QWidget)

from . import theme as T
from .common import MONO_FONT, UI_FONT, card, mono          # noqa: F401  (re-exported: other modules import card / mono from here)
from .components import StatusChip, ToastHost, TopTabs
from .fly_page import FlyPage
from .model import GcsModel, find_fc_port, list_ports, warnings
from .radio_page import RadioPage
from .shell import BellButton, CommandPalette, Logo, NotificationPopup
from .flights_page import FlightsPage
from .live_page import LiveDataPage
from .workspace import Workspace, lazy_import, load_state, save_state

# Mission-control look: greys + ONE blue accent; amber only for a caution; red only for SAFE and alarms (tokens live in theme.py).
BLUE, GREEN, AMBER, RED = T.ACC, T.TXT, T.AMB, T.RED


def apply_dark(app) -> None:
    """Global look: Fusion + dark palette from the theme tokens + the theme stylesheet."""
    app.setStyle("Fusion")
    T.load_fonts()
    pal = QPalette()
    for role, col in ((QPalette.ColorRole.Window, T.BG), (QPalette.ColorRole.Base, T.PANEL), (QPalette.ColorRole.AlternateBase, T.BG),
                      (QPalette.ColorRole.WindowText, T.TXT), (QPalette.ColorRole.Text, T.TXT), (QPalette.ColorRole.Button, T.PANEL),
                      (QPalette.ColorRole.ButtonText, T.TXT), (QPalette.ColorRole.Highlight, T.ACC), (QPalette.ColorRole.HighlightedText, T.ON_ACC),
                      (QPalette.ColorRole.ToolTipBase, T.PANEL), (QPalette.ColorRole.ToolTipText, T.TXT), (QPalette.ColorRole.PlaceholderText, T.FAINT)):
        pal.setColor(role, QColor(col))
    app.setPalette(pal)
    f = QFont(); f.setFamilies(["Lato"] + list(UI_FONT)); f.setPixelSize(13); app.setFont(f)
    app.setStyleSheet(T.qss())


class MainWindow(QMainWindow):
    """Window shell: header, navigation (five workspaces), workspace hosting, timers and connection handling."""
    WORKSPACES = ("FLY", "VIDEO", "SETUP", "ANALYZE", "DIAGNOSTICS", "SETTINGS")

    def __init__(self, model: GcsModel | None = None):
        super().__init__()
        self.model = model or GcsModel()
        self.setWindowTitle("DroneForge GCS")
        self.resize(1920, 1080); self.setMinimumSize(1366, 740)
        self._say_timer = QTimer(self); self._say_timer.setSingleShot(True); self._say_timer.timeout.connect(self._clear_say)
        self._ui = load_state(); self._seen_seq = 0
        root = QWidget(); root.setObjectName("root"); self.setCentralWidget(root); root.setFocusPolicy(Qt.FocusPolicy.ClickFocus)   # no editable widget grabs the keys at start-up
        v = QVBoxLayout(root); v.setContentsMargins(0, 0, 0, 0); v.setSpacing(0)
        v.addWidget(self._build_header())
        self.banner = QLabel(""); self.banner.setWordWrap(True)
        self.banner.setStyleSheet(f"background:{T.RED};color:#ffffff;padding:6px 24px;font-weight:700")
        self.banner.setVisible(False); v.addWidget(self.banner)
        self.status_line = QLabel(""); self.status_line.setObjectName("statusLine"); self.status_line.setContentsMargins(24, 4, 24, 0); self.status_line.setVisible(False)
        v.addWidget(self.status_line)
        self.stack = QStackedWidget(); v.addWidget(self.stack, 1)
        from .vision_bar import VisionBar
        self.vision_bar = VisionBar(self); v.addWidget(self.vision_bar)      # DEPTH / DETECT / FACE REC / TRACK / VO under every page
        self.map_frames = self.map_telem = None
        from nimbus_core.map_wire import MapClient
        self.map_client = MapClient(); self.map_client.start("127.0.0.1")      # RTAB-Map /map + /map_thin + pose (map_stream_node, tcp 5765): the Fly and Video map views read it
        try:                                                                 # the mapping pipeline (realmap/o4_map_node.py) gets the O4 frames + FC telemetry from here
            from .map_feed import FrameShare, TelemetryFeed
            from .video_source import get_source
            self.map_frames = FrameShare(lambda: get_source(self).latest_with_time()); self.map_telem = TelemetryFeed(self.model.core)
        except Exception as e:                                               # noqa: BLE001
            print("map feed unavailable:", e)
        self.workspaces: dict[str, Workspace] = {}
        self._build_workspaces()
        self.tabs.changed.connect(self._nav_changed)
        self.toasts = ToastHost(root)
        self.palette = CommandPalette(self); self.notes = NotificationPopup(self)
        self.fast = QTimer(self); self.fast.timeout.connect(self.model.tick); self.fast.start(20)       # controls at 50 Hz
        self.slow = QTimer(self); self.slow.timeout.connect(self.refresh); self.slow.start(100)         # screen at 10 Hz
        self.refresh_ports()
        self._shortcuts()
        for n, t in (self._ui.get("subtabs") or {}).items():
            if n in self.workspaces and isinstance(t, str):
                self.workspaces[n].preset(t)
        self.goto("FLY", (self._ui.get("subtabs") or {}).get("FLY"))      # 2026-11-23 (owner): the GCS always opens on FLY (each page still remembers its sub-tab)
        self._ui_ready = True
        root.setFocus()

    # --- workspaces ---------------------------------------------------------------------------
    def _build_workspaces(self) -> None:
        def add(name: str) -> Workspace:
            w = Workspace(name); self.workspaces[name] = w; self.stack.addWidget(w)
            w.changed.connect(lambda _i, n=name: self._sub_changed(n)); return w
        w = add("FLY")
        w.add_page("Fly", lambda: self._reg("fly", FlyPage(self)), pad=False, fit=True)
        w = add("VIDEO")
        w.add_page("Video", lambda: self._reg("video_page", lazy_import("video_page", "VideoPage", self, "Video")))
        w = add("SETUP")
        w.add_page("Setup", lambda: self._reg("setup_page", self._import("system_page", "SetupPage", "Setup")), scroll=False, pad=False)
        w = add("ANALYZE")
        w.add_page("Flights", lambda: self._reg("flights_page", FlightsPage(self)))
        w.add_page("Live data", lambda: self._reg("live_page", LiveDataPage(self)))
        w = add("DIAGNOSTICS")
        w.add_page("Diagnostics", lambda: self._reg("diagnostics_page", self._import("diagnostics_page", "DiagnosticsPage", "Diagnostics")))
        w = add("SETTINGS")
        w.add_page("Settings", lambda: self._reg("settings_page", self._import("settings_page", "SettingsPage", "Settings")))
        self.fly = self.workspaces["FLY"].build(0)                                  # the Fly page always exists (director, SAFE, pre-flight)

    def _import(self, module: str, cls: str, label: str) -> QWidget:
        return lazy_import(module, cls, self, label)

    def _reg(self, attr: str, page: QWidget) -> QWidget:
        setattr(self, attr, page)
        if attr == "setup_page":
            for key, _t, mod_attr, _c in page.SECTIONS:
                setattr(self, mod_attr, page.sections[key])
        if attr == "flights_page":
            self.logs_tab = page.logs
        if attr == "live_page":
            self.telemetry_page = page.telemetry
        return page

    # page attribute -> (workspace, sub-tab title): `win.sensor_tab` etc. keep working although pages are built lazily
    _PAGE_SLOTS = {"sensor_tab": ("SETUP", "Setup"), "motor_tab": ("SETUP", "Setup"), "calibration_page": ("SETUP", "Setup"), "radio_page": ("SETUP", "Setup"),
                   "logs_tab": ("ANALYZE", "Flights"), "telemetry_page": ("ANALYZE", "Live data"),
                   "settings_page": ("SETTINGS", "Settings"), "diagnostics_page": ("DIAGNOSTICS", "Diagnostics"),
                   "video_page": ("VIDEO", "Video")}

    def __getattr__(self, name: str):
        """Build-on-demand access to pages (and to widgets that live on the hosted pages, e.g. `win.summary`, `win.tel`)."""
        d = self.__dict__
        if name.startswith("_") or "workspaces" not in d:
            raise AttributeError(name)
        if name in MainWindow._PAGE_SLOTS:
            ws, title = MainWindow._PAGE_SLOTS[name]; w = d["workspaces"][ws]; page = w.build(w.index_of(title))
            return self.__dict__.get(name, page)                                    # SETUP sections are registered by _reg
        for page_attr in ("radio_page", "telemetry_page"):
            ws, title = MainWindow._PAGE_SLOTS[page_attr]; w = d["workspaces"][ws]; page = w.build(w.index_of(title))
            page = self.__dict__.get(page_attr, page)
            if name in page.__dict__:
                return page.__dict__[name]
        raise AttributeError(name)

    def build_all(self) -> None:
        """Build every page now (tests / screenshots)."""
        for w in self.workspaces.values():
            for i in range(w.count()):
                w.build(i)

    def goto(self, workspace: str, subtab: str | None = None) -> bool:
        ws = self.workspaces.get(workspace.upper())
        if ws is None:
            return False
        sections = {"Sensors": "sensors", "Motor test": "motors", "Calibration": "calibration", "Radio": "radio"}
        if workspace.upper() == "SETUP" and subtab in sections:
            page = ws.build(0); ws.select(0); page.nav.select(sections[subtab])
        elif subtab:
            if ws.index_of(subtab) < 0:
                return False
            ws.select_title(subtab)
        else:
            ws.select(ws.current_index())
        idx = self.WORKSPACES.index(workspace.upper())
        if idx != self.stack.currentIndex() and self.stack.isVisible(): self._crossfade()
        self.stack.setCurrentIndex(idx); self.tabs.blockSignals(True); self.tabs.select(idx); self.tabs.blockSignals(False); self._save_ui()
        return True

    FADE_MS = 150

    def _crossfade(self) -> None:
        """Smooth tab change (2026-11-24): a snapshot of the old page fades out over the new one (150 ms); input goes straight to the new page."""
        from PyQt6.QtCore import QPropertyAnimation, QEasingCurve
        from PyQt6.QtWidgets import QGraphicsOpacityEffect
        old = getattr(self, "_fade", None)
        if old is not None:
            try: old.deleteLater()
            except RuntimeError: pass                                        # already gone (its fade finished)
            self._fade = None
        pm = self.stack.grab()
        if pm.isNull(): return
        lab = QLabel(self.stack); lab.setPixmap(pm); lab.setGeometry(self.stack.rect()); lab.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, True)
        eff = QGraphicsOpacityEffect(lab); eff.setOpacity(1.0); lab.setGraphicsEffect(eff); lab.show(); lab.raise_()
        an = QPropertyAnimation(eff, b"opacity", lab); an.setDuration(self.FADE_MS); an.setStartValue(1.0); an.setEndValue(0.0); an.setEasingCurve(QEasingCurve.Type.OutCubic)
        an.finished.connect(lab.deleteLater); an.start(); self._fade = lab

    def current_workspace(self) -> str:
        return self.WORKSPACES[self.stack.currentIndex()]

    def _nav_changed(self, i: int) -> None:
        self.goto(self.WORKSPACES[i])

    def _sub_changed(self, name: str) -> None:
        self._save_ui()

    def _save_ui(self) -> None:
        if not getattr(self, "_ui_ready", False):
            return
        sub = dict(self._ui.get("subtabs") or {})
        for n, w in self.workspaces.items():
            sub[n] = w.current_title()
        self._ui = {"workspace": self.current_workspace(), "subtabs": sub}
        save_state(self._ui)

    def _all_pages(self):
        return [p for w in self.workspaces.values() for p in w.built_pages()]

    # --- keyboard -----------------------------------------------------------------------------
    def _shortcuts(self) -> None:
        def sc(seq, fn):
            q = QShortcut(QKeySequence(seq), self); q.setContext(Qt.ShortcutContext.WindowShortcut); q.activated.connect(fn); return q
        self.sc_safe = sc(Qt.Key.Key_Escape, self.on_safe)
        from .flight_keys import FlightKeys
        self.flight_keys = FlightKeys(self)                  # hold A = ARM, hold T = TAKE OFF, D = DISARM, L = LAND (same gates as the buttons)
        self.sc_nums = [sc(str(i + 1), lambda i=i: self._number_key(i)) for i in range(len(self.WORKSPACES))]
        self.sc_sub_next = sc("Ctrl+Tab", lambda: self.workspaces[self.current_workspace()].cycle(1))
        self.sc_sub_prev = sc("Ctrl+Shift+Tab", lambda: self.workspaces[self.current_workspace()].cycle(-1))
        self.sc_ws_prev = sc("Ctrl+Left", lambda: self._step_workspace(-1))
        self.sc_ws_next = sc("Ctrl+Right", lambda: self._step_workspace(1))
        self.sc_palette = sc("Ctrl+K", self.open_palette)

    @staticmethod
    def _editing() -> bool:
        w = QApplication.focusWidget()
        return isinstance(w, (QLineEdit, QAbstractSpinBox, QPlainTextEdit, QTextEdit)) or (isinstance(w, QComboBox) and w.isEditable())

    def _number_key(self, i: int) -> None:
        if not self._editing():
            self.goto(self.WORKSPACES[i])

    def _step_workspace(self, step: int) -> None:
        self.goto(self.WORKSPACES[(self.stack.currentIndex() + step) % len(self.WORKSPACES)])

    # --- header -------------------------------------------------------------------------------
    def _build_header(self) -> QWidget:
        """One fixed 56 px row on every page: logo, FLY VIDEO SETUP ANALYZE DIAGNOSTICS SETTINGS, source, DISARMED / MODE / BATT / LINK chips, bell, SAFE."""
        h = QFrame(); h.setObjectName("topbar"); h.setFixedHeight(T.TOP_BAR_H)
        h.setStyleSheet(f"QFrame#topbar {{ background:{T.BG}; border:none; border-bottom:1px solid {T.HAIR}; }}")
        l = QHBoxLayout(h); l.setContentsMargins(24, 0, 24, 0); l.setSpacing(8)
        l.addWidget(Logo())
        self.brand = QLabel("DRONEFORGE  GCS"); self.brand.setFont(T.f_label(13)); l.addWidget(self.brand); l.addSpacing(32)
        self.tabs = TopTabs(list(self.WORKSPACES)); l.addWidget(self.tabs)
        l.addStretch(1)
        # connection: radio port, rescan, connect, and a read-only source label
        self.port_box = QComboBox(); self.port_box.setEditable(True); self.port_box.setToolTip("Radio port")
        self.port_box.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.btn_rescan = QPushButton("Rescan"); self.btn_rescan.setToolTip("Look for the radio port again"); self.btn_rescan.clicked.connect(self.refresh_ports); self.btn_rescan.setFixedHeight(32)
        self.btn_connect = QPushButton("Connect"); self.btn_connect.setObjectName("primary"); self.btn_connect.clicked.connect(self.toggle_connection)
        self.port_box.setFixedHeight(32); self.port_box.setMinimumWidth(150); self.port_box.setMaximumWidth(230)
        self.btn_connect.setFixedHeight(32); self.btn_connect.setMinimumWidth(104)
        self.btn_connect.setToolTip("Connect to / disconnect from the radio port")
        self.src_label = QLabel("OFFLINE"); self.src_label.setObjectName("label"); self.src_label.setFont(T.f_label(12)); self.src_label.setMinimumWidth(64)
        l.addWidget(self.port_box); l.addWidget(self.btn_rescan); l.addWidget(self.btn_connect); l.addWidget(self.src_label)
        self.chip_arm = StatusChip("", "DISARMED", "lock"); self.chip_mode = StatusChip("MODE", "--", "mode")
        from .components.battery_gauge import BatteryGauge
        self.chip_batt = BatteryGauge(); self.chip_link = StatusChip("LINK", "--", "wifi")      # battery: phone-style gauge next to SAFE (2026-11-24)
        for c in (self.chip_arm, self.chip_mode, self.chip_link):
            l.addWidget(c)
        self.bell = BellButton(); self.bell.clicked.connect(self.show_notifications); l.addWidget(self.bell)
        l.addWidget(self.chip_batt)
        self.btn_safe = QPushButton("SAFE   Esc"); self.btn_safe.setObjectName("danger"); self.btn_safe.setFixedHeight(32); self.btn_safe.setMinimumWidth(112); self.btn_safe.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed); self.btn_safe.clicked.connect(self.on_safe)
        self.btn_safe.setToolTip("SAFE: disarm, throttle minimum, controls idle. Always works (Esc).")
        l.addWidget(self.btn_safe)
        return h

    def resizeEvent(self, e) -> None:
        super().resizeEvent(e); self.brand.setVisible(self.width() >= 2000)

    # --- notifications, palette --------------------------------------------------------------
    def show_notifications(self) -> None:
        n = self.model.core.notifications
        self.notes.set_notices(n.all()); n.mark_read(); self.bell.set_count(0); self.notes.show_under(self.bell)

    def palette_entries(self):
        e = [(f"Go to {w}", lambda w=w: self.goto(w)) for w in self.WORKSPACES]
        for name, ws in self.workspaces.items():
            if ws.count() > 1:
                e += [(f"Go to {name} / {t}", lambda n=name, t=t: self.goto(n, t)) for t in ws.titles()]
            if name == "SETUP":                                            # SETUP is one page with a sidebar; its sections stay reachable by name
                e += [(f"Go to SETUP / {t}", lambda t=t: self.goto("SETUP", t)) for t in ("Sensors", "Calibration", "Radio")]
        m = self.model
        e += [("Action: SAFE (disarm, controls idle)", self.on_safe), ("Action: Land", self.fly.do_land), ("Action: Disarm", self.fly.do_disarm),
              ("Action: Connect / disconnect radio", self.toggle_connection),
              ("Action: Rescan ports", self.refresh_ports),
              ("Action: Show notifications", self.show_notifications)]
        return e

    def open_palette(self) -> None:
        self.palette.set_entries(self.palette_entries()); self.palette.open_palette()

    # --- global one-line status ---------------------------------------------------------------
    def say(self, text: str, level: str = "ok") -> None:
        """One global status line under the header, any page may use it. Cleared after 12 s."""
        col = {"ok": T.TXT, "warn": T.AMB, "err": T.RED}.get(level, T.TXT)
        self.status_line.setText(text); self.status_line.setVisible(bool(text)); self.status_line.setStyleSheet(f"color:{col};font-weight:700")
        self.status_line.setToolTip(text)
        self._say_timer.start(12000) if text else self._say_timer.stop()

    def _clear_say(self) -> None:
        self.status_line.setText(""); self.status_line.setVisible(False)

    def _refresh_chips(self, s: dict) -> None:
        d = self.model.core.director.state()
        kind = "SIM" if s["sim"] else "BENCH" if s["bench"] else "LIVE" if s["connected"] else "OFFLINE"
        self.src_label.setText(kind)
        lq = s["lq"]
        if not s["connected"] or lq is None:
            self.chip_link.set_value("--")
        elif d.link_state == "lost":
            self.chip_link.set_value("LOST", "alarm")
        elif d.link_state == "degraded" or lq < 50:
            self.chip_link.set_value(f"{lq:.0f} %", "caution")
        else:
            self.chip_link.set_value(f"{lq:.0f} %")
        pct, v = s["battery_pct"], s["battery_v"]
        self.chip_batt.set_battery(None if (pct is None or v is None) else pct, v)                  # amber below 30 %: a warning only
        self.chip_mode.set_value((s["mode"] or "--").rstrip("*"))
        armed = bool(s["mode"]) and not s["mode"].endswith("*")
        self.chip_arm.set_value("ARMED" if armed else "DISARMED" if s["connected"] else "--")
        self.chip_arm._icon = "unlock" if armed else "lock"; self.chip_arm.update()
        n = self.model.core.notifications
        un = n.unread()
        if self.notes.isVisible():
            n.mark_read(); un = []
        self.bell.set_count(len(un))
        for x in un:                                                         # new caution / alarm notices also appear as a toast
            if x.seq > self._seen_seq and x.level != "info":
                self.toasts.show_toast(x.text)
        if un:
            self._seen_seq = max(self._seen_seq, un[-1].seq)

    # --- connection ---------------------------------------------------------------------------
    def set_status(self, ok: bool, text: str) -> None:
        """Colour-coded one-line status (replaces every pop-up); shown in the global status line."""
        self.say(text, "ok" if ok else "err")

    def refresh_ports(self) -> None:
        cur = self.port_box.currentText()
        self.port_box.clear(); self.port_box.addItems(list_ports())
        pref = next((p for p in list_ports() if "CP2102" in p), None)
        self.port_box.setCurrentText(cur or pref or "")

    def toggle_connection(self) -> None:
        if self.model.connected:
            self.model.disconnect(); self.set_status(True, "Disconnected."); return
        port = self.port_box.currentText().strip()
        try:
            self.model.connect(port)
        except OSError as e:
            self.set_status(False, f"Cannot connect to {port or '(no port)'}: {e.strerror or e}. Is another program using it?"); return
        self.set_status(True, f"Connected to {port}.")

    def on_safe(self) -> None:
        """SAFE: always works, from every page (also bound to Esc)."""
        self.fly.say(self.model.core.director.safe())

    def closeEvent(self, e) -> None:
        self.fast.stop(); self.slow.stop(); self.vision_bar.shutdown()
        try: self.__dict__.get("video_source") and self.video_source.stop()          # the camera thread is stopped with the window (a thread stuck in a connect ends by itself)
        except Exception: pass
        try: self.map_client.stop()                                  # the map stream thread: a daemon thread still inside recv() at interpreter exit crashed the process
        except Exception: pass
        if self.map_frames is not None: self.map_frames.close()
        self.model.disconnect(); super().closeEvent(e)

    # --- refresh ------------------------------------------------------------------------------
    def refresh(self) -> None:
        m, st = self.model, self.model.state
        s = m.status()
        m.sample()
        self.btn_connect.setText("Disconnect" if s["connected"] else "Connect")
        self.port_box.setEnabled(not s["connected"])
        warn = warnings(s, sum(st.in_count.values()), time.monotonic() - m._t0)
        if s["bench"]:
            warn = ["BENCH ARMS THE REAL FC: PROPS OFF"] + list(warn)
        self.banner.setText("   |   ".join(warn)); self.banner.setVisible(bool(warn))
        self._refresh_chips(s)
        for p in self._all_pages():
            fn = getattr(p, "refresh", None)
            if fn is None:
                continue
            try:
                fn(s)
            except TypeError:                                    # pages written with refresh() and no argument
                fn()
