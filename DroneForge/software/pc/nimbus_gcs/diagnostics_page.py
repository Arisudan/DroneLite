"""DIAGNOSTICS tab (2026-11-25): are the 16 RC channels really sent and received, at the rate the flight needs?

Left: all 16 channels, what the GCS hands to the radio (SENT), what the FC itself reports it received (FC GETS: MSP_RC read back over the radio, about 1-2 per second and ~0.5 s old, so a
moving stick differs a little), the difference, a bar with both markers, the result of the channel test (a distinctive pattern on CH9-16, run by itself once per radio session while
disarmed: nimbus_core/rc_check.py) and what each channel needs. Right: the rates that matter for stable flight, each against what it needs, with a state that reads without colour
(OK / CAUTION / FAIL). Reads only; the one action is the channel test (disarmed, cancelled the moment anything arms). Colours from theme.py only."""
from __future__ import annotations

import time
from typing import Optional

from PyQt6.QtCore import QRectF, Qt
from PyQt6.QtGui import QColor, QPainter, QPen
from PyQt6.QtWidgets import (QAbstractItemView, QHBoxLayout, QHeaderView, QLabel, QPushButton, QStyledItemDelegate, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget)

from nimbus_core import rc_check as RC
from nimbus_pc import crsf
from . import theme
from .components import Card

OK, CAUTION, FAIL, NONE = "✔ OK", "⚠ CAUTION", "✖ FAIL", "--"       # state readable without colour
SHORT_NEEDS = {1: "100 Hz", 2: "100 Hz", 3: "100 Hz", 4: "100 Hz", 5: "100 Hz, < 0.25 s", 6: "held", 7: "held", 8: "held", 9: "on change, 10-bit", 10: "held", 11: "10 Hz", 12: "-", 13: "10 Hz, 10-bit", 14: "10 Hz, 10-bit", 15: "10 Hz", 16: "-"}   # full text in the tooltip
TOL_US = RC.EXACT_US


def packet_hz(st) -> Optional[float]:
    """The ELRS radio packet rate from the TX's RADIO_ID (0x3A) timing frame (interval in 0.1 us)."""
    p = st.last_payload.get(0x3A)
    if not p or len(p) < 11 or p[2] != 0x10:
        return None
    interval = int.from_bytes(p[3:7], "big") / 10.0
    return 1e6 / interval if interval else None


def judge_rate(value: Optional[float], ok: float, caution: float) -> str:
    """OK at or above `ok`, CAUTION at or above `caution`, FAIL below; NONE when unknown."""
    if value is None: return NONE
    return OK if value >= ok else (CAUTION if value >= caution else FAIL)


def judge_low(value: Optional[float], ok: float, caution: float) -> str:
    """For 'smaller is better' numbers (gaps, trips)."""
    if value is None: return NONE
    return OK if value <= ok else (CAUTION if value <= caution else FAIL)


class _Bar(QStyledItemDelegate):
    """988..2012 us track with the SENT marker (accent) and the FC marker (text colour)."""
    def paint(self, p: QPainter, opt, idx) -> None:
        data = idx.data(Qt.ItemDataRole.UserRole)
        r = opt.rect.adjusted(6, 0, -6, 0); y = r.center().y()
        p.save(); p.setPen(QPen(QColor(theme.HAIR2), 2)); p.drawLine(r.left(), y, r.right(), y)
        for us, col, h in ((data[0] if data else None, theme.ACC, 7), (data[1] if data else None, theme.TXT, 4)):
            if us is None: continue
            x = r.left() + (min(max(us, 988), 2012) - 988) / 1024.0 * r.width()
            p.setPen(QPen(QColor(col), 3 if h == 7 else 2)); p.drawLine(int(x), y - h, int(x), y + h)
        p.restore()


def _item(text: str = "", color: str = theme.TXT, align=Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignLeft) -> QTableWidgetItem:
    it = QTableWidgetItem(text); it.setForeground(QColor(color)); it.setTextAlignment(align); return it


def _state_color(state: str) -> str:
    return theme.TXT if state == OK else (theme.AMB if state == CAUTION else (theme.RED if state == FAIL else theme.DIM))


class DiagnosticsPage(QWidget):
    RATES = ("rc_tx", "gap", "stale", "air", "state", "batt", "lq", "readback", "aid")

    def __init__(self, win) -> None:
        super().__init__()
        self.win = win
        mono = theme.f_mono(12)
        # ---- channels
        self.tbl = t = QTableWidget(16, 8); t.setFont(mono)
        t.setHorizontalHeaderLabels(["CH", "NAME", "GCS SENDS", "FC GETS", "DIFF", "SENT / FC", "TEST", "NEEDS"])
        t.verticalHeader().setVisible(False); t.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers); t.setSelectionMode(QAbstractItemView.SelectionMode.NoSelection)
        t.setShowGrid(False); t.setAlternatingRowColors(True); t.verticalHeader().setDefaultSectionSize(26); t.setWordWrap(False); t.setTextElideMode(Qt.TextElideMode.ElideRight)
        t.setItemDelegateForColumn(5, _Bar(t))
        for i in range(16):
            ch = i + 1; name, what, need = RC.ROLES[ch]
            t.setItem(i, 0, _item(str(ch), theme.DIM, Qt.AlignmentFlag.AlignCenter)); n = _item(name); n.setToolTip(what); t.setItem(i, 1, n)
            for c in (2, 3, 4, 6): t.setItem(i, c, _item("--", theme.DIM, Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignRight if c != 6 else Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignLeft))
            t.setItem(i, 5, QTableWidgetItem()); nd = _item(SHORT_NEEDS[ch], theme.DIM); nd.setToolTip(f"{what}\nNeeds: {need}"); t.setItem(i, 7, nd)
        for c, w in ((0, 34), (1, 88), (2, 100), (3, 88), (4, 48), (5, 104), (6, 112)): t.setColumnWidth(c, w)
        t.horizontalHeader().setSectionResizeMode(7, QHeaderView.ResizeMode.Stretch)
        self.ch_card = Card("RC channels 1-16", "GCS SENDS -> radio -> FC GETS (read back over the radio)"); self.ch_card.layout().setContentsMargins(16, 16, 16, 16); self.ch_card.body.addWidget(t, 1)
        row = QHBoxLayout(); row.setSpacing(8)
        self.btn_test = QPushButton("Run channel test"); self.btn_test.setObjectName("primary"); self.btn_test.clicked.connect(self.run_test)
        self.btn_reset = QPushButton("Reset counters"); self.btn_reset.clicked.connect(self.reset_counters)
        self.test_line = QLabel(""); self.test_line.setWordWrap(True)
        row.addWidget(self.btn_test); row.addWidget(self.btn_reset); row.addWidget(self.test_line, 1); self.ch_card.body.addLayout(row)
        # ---- rates
        self.rates = r = QTableWidget(len(self.RATES), 4); r.setFont(mono); r.setHorizontalHeaderLabels(["WHAT", "NOW", "NEEDS", "STATE"])
        r.verticalHeader().setVisible(False); r.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers); r.setSelectionMode(QAbstractItemView.SelectionMode.NoSelection)
        r.setShowGrid(False); r.setAlternatingRowColors(True); r.verticalHeader().setDefaultSectionSize(26); r.setWordWrap(False); r.setTextElideMode(Qt.TextElideMode.ElideRight)
        for i in range(len(self.RATES)):
            for c in range(4): r.setItem(i, c, _item("--", theme.DIM if c else theme.TXT))
        for c, w in ((0, 236), (1, 112), (3, 100)): r.setColumnWidth(c, w)
        r.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeMode.Stretch)
        self.rate_card = Card("Rates", "NOW against what stable flight NEEDS"); self.rate_card.layout().setContentsMargins(16, 16, 16, 16); self.rate_card.body.addWidget(r, 1)
        self.note = QLabel("The FC read-back arrives 1-2 times a second and is about 0.5 s old: while a stick moves it differs a little. The channel test sends a fixed pattern on CH9-16 while disarmed, so the read-back is exact.")
        self.note.setWordWrap(True); self.note.setStyleSheet(f"color:{theme.DIM}"); self.rate_card.body.addWidget(self.note)
        lay = QHBoxLayout(self); lay.setContentsMargins(0, 0, 0, 0); lay.setSpacing(theme.SP["m"]); lay.addWidget(self.ch_card, 5); lay.addWidget(self.rate_card, 4)
        self._sig = None

    # ---- actions
    def run_test(self) -> None:
        core = self.win.model.core
        self.test_line.setText("Channel test started: a pattern goes out on CH9-16 for a few seconds (the drone stays disarmed)." if core.start_rc_check()
                               else "The channel test needs the live radio link, a disarmed drone and no flight in progress.")

    def reset_counters(self) -> None:
        sender = getattr(self.win.model.core.sess, "sender", None)
        if sender is not None:
            sender.max_interval_s = 0.0; sender.stale_trips = 0; sender._gaps.clear() if hasattr(sender, "_gaps") else None

    # ---- live
    def refresh(self, s: dict) -> None:
        if not self.isVisible():
            return
        core = self.win.model.core; st = self.win.model.state; sess = core.sess; sender = getattr(sess, "sender", None); probe = core.probe
        live = bool(sess is not None and getattr(core, "state_source", "") == "live")
        # sent channels (ticks -> us): the real sender's newest frame, else what the simulator was handed
        sent = None
        if sender is not None and getattr(sender, "last_channels", None) is not None and getattr(sender, "frames_sent", 0):
            sent = [RC.ticks_to_us(v) for v in sender.last_channels]
        elif getattr(core, "last_sent", None) is not None:
            try: sent = [RC.ticks_to_us(v) for v in core.last_sent.to_channels(bool(getattr(core, "allow_arm", False)))]
            except Exception: sent = None
        fc = list(probe.rc) if (probe is not None and probe.rc) else None
        fc_age = (time.monotonic() - probe.t_rc) if (probe is not None and probe.t_rc) else None
        fc_ok = fc is not None and fc_age is not None and fc_age < 5.0
        verdict = {r[0]: r for r in core.rc_check.rows} if core.rc_check.rows else {}
        for i in range(16):
            ch = i + 1; t = self.tbl
            sv = sent[i] if sent and i < len(sent) else None; fv = fc[i] if fc_ok and i < len(fc) else None
            err = None if sv is None or fv is None else fv - sv
            t.item(i, 2).setText("--" if sv is None else f"{sv:.0f} us"); t.item(i, 3).setText("--" if fv is None else f"{fv} us")
            e = t.item(i, 4); e.setText("--" if err is None else f"{round(err):+d}")
            e.setForeground(QColor(theme.DIM if err is None else (theme.TXT if abs(err) <= TOL_US else theme.AMB)))
            t.item(i, 5).setData(Qt.ItemDataRole.UserRole, (sv, fv))
            v = verdict.get(ch)
            if v is None: tx, col = "--", theme.DIM
            elif v[4] == "ok": tx, col = f"✔ {v[3]:+d} us", theme.TXT
            elif v[4] == "coarse": tx, col = f"⚠ coarse {v[3]:+d} us", theme.AMB
            else: tx, col = f"✖ {v[4]}" + ("" if v[3] is None else f" {v[3]:+d} us"), theme.RED
            it = t.item(i, 6); it.setText(tx); it.setForeground(QColor(col))
        self.tbl.viewport().update()
        chk = core.rc_check
        sym = {"ok": "✔", "running": "…", "fail": "✖", "silent": "⚠", "idle": "--"}.get(chk.state, "--")
        self.test_line.setText(f"{sym} Channel test: {chk.detail}"); self.test_line.setStyleSheet(f"color:{theme.RED if chk.state == 'fail' else (theme.AMB if chk.state == 'silent' else theme.TXT)}")
        self.btn_test.setEnabled(live and not getattr(core.autonomy, "fc_armed", False) and not getattr(core.autonomy, "busy", False) and chk.state != "running")
        # ---- rates
        tx_hz = st.rate(st.out_times, crsf.TYPE_RC_CHANNELS) if live else None
        gap = (getattr(sender, "recent_max_gap_s", None) * 1000.0) if (live and sender is not None and getattr(sender, "recent_max_gap_s", None) is not None) else None
        trips = getattr(sender, "stale_trips", None) if live else None
        air = packet_hz(st) if live else None
        state_hz = st.rate(st.in_times, crsf.TYPE_STATE) if live else None
        batt_hz = st.rate(st.in_times, crsf.TYPE_BATTERY) if live else None
        link = st.decoded.get("link") if live and isinstance(getattr(st, "decoded", None), dict) else None
        lq = None if not link else min(link.get("up_lq", 0), link.get("down_lq", 0))
        rb_hz = probe.rc_reply_hz if (live and probe is not None) else None
        aid = getattr(core, "pos_aid", None); aid_age = None if not aid else time.monotonic() - aid[2]
        rows = {
            "rc_tx": ("RC frames: GCS to radio", None if tx_hz is None else f"{tx_hz:.0f} Hz", "100 Hz", judge_rate(tx_hz, 90, 50)),
            "gap": ("Longest gap between RC frames (5 s)", None if gap is None else f"{gap:.0f} ms", "< 250 ms", judge_low(gap, 30, 249)),
            "stale": ("Stale-input failsafe trips", None if trips is None else f"{trips}", "0", judge_low(trips, 0, 2)),
            "air": ("Radio packet rate over the air", None if air is None else f"{air:.0f} Hz", "100 Hz Full", NONE if air is None else (OK if 95 <= air <= 105 else CAUTION)),
            "state": ("FC state frame 0xD6 (FC to GCS)", None if state_hz is None else f"{state_hz:.1f} Hz", "12-18 Hz", judge_rate(state_hz, 12, 6)),
            "batt": ("FC battery frame", None if batt_hz is None else f"{batt_hz:.1f} Hz", "about 1 Hz", judge_rate(batt_hz, 0.5, 0.2)),
            "lq": ("Link quality (worse of up / down)", None if lq is None else f"{lq:.0f} %", ">= 80 %", judge_rate(lq, 80, 50)),
            "readback": ("Channel read-back from the FC (MSP_RC)", None if rb_hz is None else f"{rb_hz:.1f} Hz" + ("" if probe is None or probe.rc_rtt_s is None else f", {probe.rc_rtt_s * 1000:.0f} ms"), ">= 0.5 Hz", judge_rate(rb_hz, 0.5, 0.05)),
            "aid": ("VO aid on CH13-15", "not sent" if aid is None or aid_age is None or aid_age > 0.25 else f"fresh {aid_age * 1000:.0f} ms", "10 Hz, < 0.25 s", NONE if aid is None else (OK if aid_age is not None and aid_age <= 0.25 else CAUTION)),
        }
        for i, key in enumerate(self.RATES):
            label, now, need, state = rows[key]; tb = self.rates
            tb.item(i, 0).setText(label); tb.item(i, 1).setText("--" if now is None else now); tb.item(i, 2).setText(need)
            si = tb.item(i, 3); si.setText(state); si.setForeground(QColor(_state_color(state)))
