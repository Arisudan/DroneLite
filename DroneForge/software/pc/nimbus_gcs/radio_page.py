"""SETUP > Radio: link status LEDs, the devices found on the link and the RF timing."""
from __future__ import annotations

from PyQt6.QtWidgets import QGridLayout, QLabel, QWidget

from . import theme
from .components import Card
from .widgets import BAD, DIM, GOOD, WARN, Led


class RadioPage(QWidget):
    def __init__(self, win) -> None:
        super().__init__()
        self.win = win
        s = Card("Radio link"); sl = s.body
        self.led = {k: Led(k) for k in ("Connection", "Radio link", "Telemetry", "Sender", "Failsafe", "Watchdog", "Arm channel", "Battery")}
        lg = QGridLayout(); lg.setHorizontalSpacing(10); lg.setVerticalSpacing(4)
        for i, led in enumerate(self.led.values()):
            lg.addWidget(led, i // 2, i % 2)
        sl.addLayout(lg)
        d = Card("Devices"); dl = d.body
        self.devices = QLabel("no DEVICE_INFO yet"); self.devices.setObjectName("mono"); self.devices.setWordWrap(True); dl.addWidget(self.devices)
        tm = Card("RF timing"); tml = tm.body
        self.rf = QLabel("--"); self.rf.setObjectName("mono"); self.rf.setWordWrap(True); tml.addWidget(self.rf)
        g = QGridLayout(self); g.setContentsMargins(0, 0, 0, 0); g.setSpacing(16)
        from PyQt6.QtCore import Qt
        g.addWidget(s, 0, 0, Qt.AlignmentFlag.AlignTop); g.addWidget(d, 0, 1, Qt.AlignmentFlag.AlignTop); g.addWidget(tm, 1, 0, 1, 2, Qt.AlignmentFlag.AlignTop)
        g.setRowStretch(2, 1); g.setColumnStretch(0, 1); g.setColumnStretch(1, 1)

    def refresh(self, s: dict) -> None:
        m = self.win.model; st = m.state; L = self.led; lq = s["lq"]; pct = s["battery_pct"]
        L["Connection"].set(GOOD if s["connected"] else DIM, s["port"].split("/")[-1][:34] if s["connected"] else "not connected")
        L["Radio link"].set(DIM if lq is None else GOOD if lq >= 90 else WARN if lq >= 50 else BAD, "" if lq is None else f"LQ {lq}%")
        L["Telemetry"].set(GOOD if (s["connected"] and s["no_data_s"] < 1.5 and st.decoded.get("battery")) else DIM if not s["connected"] else WARN)
        L["Sender"].set(DIM if not s["connected"] else GOOD if s["sender_alive"] else BAD, f"{s['fps']:.0f}/s" if s["connected"] else "")
        L["Failsafe"].set(BAD if s["failsafe"] else GOOD if s["connected"] else DIM, "ON" if s["failsafe"] else "ok")
        L["Watchdog"].set(DIM if s["watchdog"] == "off" else GOOD if s["watchdog"] == "ok" else BAD, s["watchdog"])
        L["Arm channel"].set(BAD if s["arm_high"] else GOOD if s["connected"] else DIM, "HIGH" if s["arm_high"] else "low")
        L["Battery"].set(DIM if pct is None else GOOD if pct >= 30 else WARN if pct >= 15 else BAD, "" if pct is None else f"{pct:.0f}%  {s['battery_cell_v']:.2f} V/cell")
        names = {0xEE: "TX", 0xEC: "RX", 0xC8: "FC"}
        self.devices.setText("\n".join(f"{names.get(o, hex(o))}: {d.get('name', '?')}" for o, d in sorted(st.device_info.items())) or "no DEVICE_INFO yet")
        self.rf.setText(s["rf_timing"] or "--")
