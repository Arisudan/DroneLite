"""SETUP: one page with a sidebar over the Sensors / Motor test / Calibration / Radio sections (the Diagnostics page was removed on 2026-11-21, archived in
archive/gcs_removed_2026-11-21; the Motor test was removed then too and restored on 2026-11-23 at the owner's request). No dialogs."""
from __future__ import annotations

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QVBoxLayout, QWidget


class SetupPage(QWidget):
    """SETUP as one page: QGC-style sidebar (status dot + text state) over the Sensors / Calibration / Radio sections."""

    SECTIONS = (("sensors", "Sensors", "sensor_tab", "SensorTab"), ("motors", "Motor test", "motor_tab", "MotorTab"),
                ("calibration", "Calibration", "calibration_page", "CalibrationPage"), ("radio", "Radio", "radio_page", "RadioPage"))

    def __init__(self, win) -> None:
        super().__init__()
        import importlib
        from .components.sidebar_nav import SectionStack
        self.win = win
        self.setObjectName("body"); self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.sections: dict[str, QWidget] = {}
        lay = QVBoxLayout(self); lay.setContentsMargins(0, 0, 0, 0)
        self.hub = SectionStack(); lay.addWidget(self.hub)
        for key, title, mod, cls in self.SECTIONS:
            page = getattr(importlib.import_module(f"{__package__}.{mod}"), cls)(win)
            self.sections[key] = page; self.hub.add_section(key, title, page, "setup")
        self.nav = self.hub.nav

    def refresh(self, s: dict | None = None) -> None:
        for key, page in self.sections.items():                      # every section stays live, shown or not (as separate sub-tabs did)
            try:
                (page.refresh(s) if s is not None else None) if key == "radio" else page.refresh()
            except TypeError:
                page.refresh(s)
        try:
            m = self.win.model; st = m.state
            fresh = lambda n: (st.age(n) or 99.0) < 1.0
            self.nav.set_state("sensors", "ok" if fresh("optrange") and fresh("kinematic") else "setup")
            self.nav.set_state("calibration", "ok" if abs(float(m.core.autonomy.settings.battery_scale) - 1.0) > 1e-9 else "caution")
            self.nav.set_state("radio", "ok" if m.connected else "setup")
        except Exception:
            pass
