"""LOGS page: the FC's flight logs (SD card) shown in the GCS. Download them to the PC and delete them from the card, from here.

Flow: plug the FC into this PC with USB (disarmed, battery optional) -> "Open log mode" (the FC restarts and presents its SD card as a drive; it cannot fly or send
telemetry meanwhile) -> list, Download, Delete (only logs with a verified copy on the PC) -> "Exit log mode" (ejects the drive; the FC restarts normally).
All the rules live in fc_logs.py (unit-tested)."""
from __future__ import annotations

import os
import subprocess
import threading
import time
from typing import List, Optional

from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtWidgets import (QAbstractItemView, QFileDialog, QHBoxLayout, QHeaderView, QLabel, QScrollArea, QProgressBar, QPushButton, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget)

from . import fc_logs as FL
from . import theme
from .analysis_page import AnalysisView
from .components import Card

NORMAL, MSC, BUSY = "normal", "msc", "busy"


def human(n: int) -> str:
    return f"{n} B" if n < 1024 else f"{n / 1024:.1f} kB" if n < 1 << 20 else f"{n / (1 << 20):.1f} MB"


class LogsTab(QWidget):
    sig_status = pyqtSignal(str)
    sig_state = pyqtSignal(str)
    sig_progress = pyqtSignal(int, int)
    sig_refresh = pyqtSignal()

    def __init__(self, win) -> None:
        super().__init__()
        self.win = win
        self.state = NORMAL
        self.card: Optional[FL.Card] = None
        self.files: List[FL.LogFile] = []
        l = QVBoxLayout(self); l.setContentsMargins(0, 0, 0, 0); l.setSpacing(theme.SP["m"])
        # card 1: FC link / log mode
        cm = Card("Log mode"); self.mode_card = cm
        self.lbl = QLabel(""); self.lbl.setWordWrap(True); self.lbl.setObjectName("mono"); self.lbl.setFont(theme.f_mono(13)); cm.body.addWidget(self.lbl)
        row = QHBoxLayout(); row.setSpacing(theme.SP["s"])
        self.btn_enter = QPushButton("Open log mode"); self.btn_enter.setObjectName("primary"); self.btn_enter.clicked.connect(self.on_enter)
        self.btn_enter.setToolTip("Restarts the FC so that its SD card appears on this PC as a drive. The FC cannot fly or send telemetry until you press Exit log mode. Disarmed only.")
        self.btn_exit = QPushButton("Exit log mode"); self.btn_exit.clicked.connect(self.on_exit)
        self.btn_list = QPushButton("Refresh list"); self.btn_list.clicked.connect(self.reload)
        for b in (self.btn_enter, self.btn_list, self.btn_exit):
            b.setMinimumHeight(40); row.addWidget(b)
        row.addStretch(1); cm.body.addLayout(row)
        # card 2: the log table and its actions
        ct = Card("Flight logs on the SD card"); self.logs_card = ct
        self.table = QTableWidget(0, 5); self.table.setHorizontalHeaderLabels(["Log", "Size", "Date", "On the PC", "Delete from card"])
        self.table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows); self.table.setSelectionMode(QAbstractItemView.SelectionMode.ExtendedSelection)
        self.table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers); self.table.verticalHeader().setVisible(False)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch); self.table.setMinimumHeight(96)
        self.table.setFont(theme.f_mono(12)); self.table.setAlternatingRowColors(True); self.table.setShowGrid(False)
        ct.body.addWidget(self.table, 1)
        row2 = QHBoxLayout(); row2.setSpacing(theme.SP["s"])
        self.btn_dl = QPushButton("Download selected"); self.btn_dl.clicked.connect(lambda: self.on_download(False))
        self.btn_dl_all = QPushButton("Download new"); self.btn_dl_all.clicked.connect(lambda: self.on_download(True))
        self.btn_del = QPushButton("Delete from card"); self.btn_del.clicked.connect(self.on_delete)
        self.btn_del.setToolTip("Only logs whose copy on the PC was verified (SHA-256) can be deleted. Empty logs can always be deleted.")
        self.btn_open = QPushButton("Open PC folder"); self.btn_open.clicked.connect(self.on_open_folder)
        self.btn_an = QPushButton("Analyze selected"); self.btn_an.setObjectName("primary"); self.btn_an.clicked.connect(self.on_analyze)
        self.btn_an.setToolTip("Decodes the selected log (from the card, or its saved copy) and prints what happened: motor steps, battery, range, PID, disarm reason.")
        self.btn_file = QPushButton("Analyze a saved log..."); self.btn_file.clicked.connect(self.on_analyze_file)
        self.btn_file.setToolTip("Pick a .BFL log already saved on the PC. Works without the FC connected.")
        for b in (self.btn_dl, self.btn_dl_all, self.btn_open, self.btn_an, self.btn_file):
            b.setMinimumHeight(40); row2.addWidget(b)
        row2.addStretch(1); self.btn_del.setMinimumHeight(40); row2.addWidget(self.btn_del)
        ct.body.addLayout(row2)
        self.bar = QProgressBar(); self.bar.setVisible(False); self.bar.setMaximumHeight(8); ct.body.addWidget(self.bar)
        self.msg = QLabel(""); self.msg.setWordWrap(True); self.msg.setFont(theme.f_body(13)); ct.body.addWidget(self.msg)
        top = QHBoxLayout(); top.setSpacing(theme.SP["m"]); top.addWidget(cm, 1); l.addLayout(top)
        self.report = AnalysisView()
        l.addWidget(ct, 1); l.addWidget(self.report, 2)
        self.sig_status.connect(self._set_msg); self.sig_state.connect(self._set_state); self.sig_progress.connect(self._set_progress); self.sig_refresh.connect(self.reload)
        self.timer = QTimer(self); self.timer.timeout.connect(self.refresh); self.timer.start(1000)
        self.refresh()

    # ---- state ---------------------------------------------------------------------------------
    def _set_msg(self, t: str) -> None:
        self._say(t)

    def _say(self, t: str, level: str = "") -> None:
        if not level:
            low = t.lower()
            level = ("error" if low.startswith(("refused", "error", "could not")) or "did not" in low or "could not" in low else
                     "warning" if any(k in low for k in ("problems", "disappeared", "nothing", "select", "plug", "not deleted", "no fc", "in progress")) else "ok")
        col = {"ok": theme.TXT, "warning": theme.AMB, "error": theme.RED}[level]
        self.msg.setText(t); self.msg.setStyleSheet(f"color:{col}")

    def _set_state(self, s: str) -> None:
        self.state = s; self._apply()

    def _set_progress(self, done: int, total: int) -> None:
        self.bar.setVisible(total > 0 and done < total); self.bar.setMaximum(max(total, 1)); self.bar.setValue(done)

    def _autonomy_idle(self) -> bool:
        return self.win.model.core.autonomy.phase == "idle"

    def refresh(self) -> None:
        port = FL.fc_serial_port()
        if self.state == MSC and self.card and (not self.card.mountpoint or not os.path.isdir(self.card.mountpoint)):
            self.card = None; self.files = []; self._fill(); self.state = NORMAL; self._say("The SD card drive disappeared (FC unplugged or restarted).")
        if self.state == NORMAL:
            self.lbl.setText(("FC found on USB (" + os.path.basename(port) + "). Press 'Open log mode' to see its logs." if port else
                              "No FC on USB. Plug the FC into this PC with its USB cable (flight controller powered, disarmed)."))
        elif self.state == MSC:
            self.lbl.setText(f"LOG MODE: SD card at {self.card.mountpoint if self.card else '?'}. The FC is not flying or sending telemetry until you press 'Exit log mode'.")
        self._apply()

    def _apply(self) -> None:
        port = FL.fc_serial_port(); busy = self.state == BUSY; msc = self.state == MSC
        self.btn_enter.setEnabled(not busy and not msc and port is not None)
        self.btn_exit.setEnabled(msc and not busy)
        for b in (self.btn_list, self.btn_dl, self.btn_dl_all, self.btn_del):
            b.setEnabled(msc and not busy)
        self.btn_an.setEnabled(msc and not busy); self.btn_open.setEnabled(True); self.btn_file.setEnabled(not busy)

    # ---- table ----------------------------------------------------------------------------------
    def reload(self) -> None:
        if not (self.card and self.card.mountpoint and os.path.isdir(self.card.mountpoint)):
            return
        self.files = FL.list_logs(self.card.mountpoint); self._fill()
        total = sum(f.size for f in self.files)
        self._say(f"{len(self.files)} log files, {human(total)} on the card.")

    def _fill(self) -> None:
        self.table.setRowCount(len(self.files))
        for i, f in enumerate(self.files):
            info = FL.saved_info(f); ok, why = FL.can_delete(f) if f.size == 0 or info else (False, "download first")
            vals = [f.name, human(f.size), time.strftime("%Y-%m-%d %H:%M", time.localtime(f.mtime)),
                    "empty (nothing to save)" if f.size == 0 else ("saved" if info else "not saved"), "yes" if ok else "no: " + why]
            for c, v in enumerate(vals):
                it = QTableWidgetItem(v); self.table.setItem(i, c, it)

    def _selected(self, only_unsaved: bool) -> List[FL.LogFile]:
        if only_unsaved:
            return [f for f in self.files if f.size > 0 and not FL.saved_info(f)]
        rows = sorted({ix.row() for ix in self.table.selectedIndexes()})
        return [self.files[r] for r in rows if r < len(self.files)]

    # ---- actions ---------------------------------------------------------------------------------
    def on_enter(self) -> None:
        port = FL.fc_serial_port()
        if port is None:
            self._say("No FC on USB: plug the FC into this PC with its USB cable first.", "warning"); return
        if not self._autonomy_idle():
            self._say("A flight or arming is in progress. Land and disarm first.", "warning"); return
        self.state = BUSY; self._apply(); self._say("Checking that the FC is disarmed...")
        threading.Thread(target=self._enter_worker, args=(port,), daemon=True).start()

    def _enter_worker(self, port: str) -> None:
        try:
            dis, why = FL.fc_is_disarmed(port)
            if dis is not True:
                self.sig_status.emit(f"Refused: the FC must be reported DISARMED ({why})."); self.sig_state.emit(NORMAL); return
            before = [p.get("path") for p in FL.usb_partitions()]
            self.sig_status.emit("Asking the FC to restart in mass-storage mode...")
            ok, msg = FL.enter_mass_storage(port)
            if not ok:
                self.sig_status.emit(msg); self.sig_state.emit(NORMAL); return
            end = time.time() + 30; card = None
            while time.time() < end and card is None:
                time.sleep(1.0); self.sig_status.emit("Waiting for the SD card drive to appear...")
                card = FL.find_card(before)
            if card is None:
                self.sig_status.emit("The SD card drive did not appear. Unplug and replug the FC USB cable to return it to normal."); self.sig_state.emit(NORMAL); return
            ok, where = FL.mount(card)
            if not ok:
                self.sig_status.emit("Found the drive but could not mount it: " + where); self.card = card; self.sig_state.emit(NORMAL); return
            self.card = card
            self.sig_state.emit(MSC); self.sig_refresh.emit()
        except Exception as e:
            self.sig_status.emit(f"Error: {e}"); self.sig_state.emit(NORMAL)

    def on_exit(self) -> None:
        if self.card is None:
            self.state = NORMAL; self._apply(); return
        self.state = BUSY; self._apply(); self._say("Ejecting the SD card and restarting the FC...")
        threading.Thread(target=self._exit_worker, args=(self.card,), daemon=True).start()

    def _exit_worker(self, card: FL.Card) -> None:
        try:
            FL.eject(card)
            end = time.time() + 30; port = None
            while time.time() < end and port is None:
                time.sleep(1.0); port = FL.fc_serial_port()
            self.card = None; self.files = []
            self.sig_status.emit("The FC is back in normal mode." if port else "Ejected. The FC did not reappear by itself: unplug and replug its USB cable (the FC stays in log mode until then).")
            self.sig_state.emit(NORMAL); self.sig_refresh.emit()
        except Exception as e:
            self.sig_status.emit(f"Error: {e}"); self.sig_state.emit(MSC)

    def on_download(self, only_unsaved: bool) -> None:
        sel = self._selected(only_unsaved)
        sel = [f for f in sel if f.size > 0]
        if not sel:
            self._say("Nothing to download (select non-empty logs, or everything is already saved)."); return
        self.state = BUSY; self._apply()
        threading.Thread(target=self._download_worker, args=(sel,), daemon=True).start()

    def _download_worker(self, sel: List[FL.LogFile]) -> None:
        okn, bad = 0, []
        for i, f in enumerate(sel):
            self.sig_status.emit(f"Downloading {f.name} ({i + 1}/{len(sel)})...")
            ok, info = FL.download(f, progress=lambda d, t: self.sig_progress.emit(d, t))
            if ok:
                okn += 1
            else:
                bad.append(f"{f.name}: {info}")
        self.sig_progress.emit(0, 0)
        self.sig_status.emit(f"Downloaded and verified {okn} of {len(sel)} logs to {FL.DEST_ROOT}." + ("  PROBLEMS: " + "; ".join(bad) if bad else ""))
        self.sig_state.emit(MSC); self.sig_refresh.emit()

    def on_delete(self) -> None:
        sel = self._selected(False)
        if not sel or not (self.card and self.card.mountpoint):
            self._say("Select the logs to delete first."); return
        ok_files = [f for f in sel if FL.can_delete(f)[0]]; refused = [f for f in sel if f not in ok_files]
        if not ok_files:
            self._say(f"Nothing deleted: none of the {len(sel)} selected log(s) has a verified copy on the PC. Download them first.", "warning"); return
        done, notes = 0, []
        for f in ok_files:
            ok, why = FL.delete(f, self.card.mountpoint)
            done += ok
            if not ok:
                notes.append(f"{f.name}: {why}")
        self.reload()
        skipped = f"  {len(refused)} log(s) not deleted (no verified copy)." if refused else ""
        self._say(f"Deleted {done} of {len(ok_files)} log(s) from the SD card." + skipped + ("  " + "; ".join(notes) if notes else ""), "warning" if (refused or notes) else "ok")

    def on_analyze(self) -> None:
        sel = self._selected(False)
        if not sel:
            self._say("Select a log first."); return
        f = sel[0]; info = FL.saved_info(f)
        path = info["saved"] if info else f.path
        self._analyze(path, f.size)

    def on_analyze_file(self) -> None:
        os.makedirs(FL.DEST_ROOT, exist_ok=True)
        path, _ = QFileDialog.getOpenFileName(self, "Open a saved flight log", FL.DEST_ROOT, "Betaflight logs (*.BFL *.bfl *.BBL *.bbl);;All files (*)")
        if path:
            self._analyze(path, os.path.getsize(path))

    def _analyze(self, path: str, size: int) -> None:
        self.report.show_log(path)

    def on_open_folder(self) -> None:
        os.makedirs(FL.DEST_ROOT, exist_ok=True)
        try:
            subprocess.Popen(["xdg-open", FL.DEST_ROOT])
        except OSError:
            self._say("Logs are saved in " + FL.DEST_ROOT)
