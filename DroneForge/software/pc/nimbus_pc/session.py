"""One object that ties the serial link, 100 Hz sender, telemetry reader, command frames, MSP requests, the sensor
watchdog and an optional CSV log together. This is the PC-side application core; the CLI is a thin layer on top."""
from __future__ import annotations

import csv
import json
import threading
import time
from typing import Callable, Optional

from . import commands
from .crsf import ADDR_FC, ADDR_RX, ADDR_TX, TYPE_OPTRANGE, TYPE_STATE, Frame
from .sender import Controls, CrsfSender, SerialLink, TelemetryReader
from .watchdog import SensorWatchdog


class TelemetryLog:
    """CSV with one row per decoded telemetry item: seconds since start, kind, JSON values."""

    def __init__(self, path: str):
        self._f = open(path, "w", newline="")
        self._w = csv.writer(self._f)
        self._w.writerow(["t_s", "kind", "values_json"])
        self._t0 = time.monotonic()
        self._lock = threading.Lock()

    def write(self, kind: str, values: dict) -> None:
        with self._lock:
            self._w.writerow([f"{time.monotonic() - self._t0:.3f}", kind, json.dumps(values, default=str)])
            self._f.flush()

    def close(self) -> None:
        with self._lock:
            self._f.close()


class Session:
    def __init__(self, port: str, hz: float = 100.0, allow_arm: bool = False, ping: bool = True,
                 watch_optrange: bool = False, listen_only: bool = False, log_path: Optional[str] = None,
                 on_telemetry: Optional[Callable[[str, dict], None]] = None,
                 on_frame: Optional[Callable[[Frame], None]] = None):
        self.link = SerialLink(port)
        self.listen_only = listen_only
        self.log = TelemetryLog(log_path) if log_path else None
        self.on_telemetry = on_telemetry
        self.on_frame = on_frame                  # every incoming CRSF frame, decoded or not
        self.assembler = commands.MspResponseAssembler()
        self._msp_responses: dict[int, dict] = {}
        self._msp_cv = threading.Condition()
        self.watchdog = SensorWatchdog() if watch_optrange else None
        self.sender = CrsfSender(self.link, hz=hz, allow_arm=allow_arm, watchdog=self.watchdog,
                                 ping_targets=(ADDR_TX, ADDR_RX, ADDR_FC) if ping else ())
        self._link_up = False
        self.reader = TelemetryReader(self.link, on_update=self._on_update, on_frame=self._on_frame)

    # --- callbacks from the reader thread ---
    def _on_frame(self, frame: Frame) -> None:
        if self.on_frame is not None:
            self.on_frame(frame)
        if self.watchdog is not None and frame.type in (TYPE_OPTRANGE, TYPE_STATE):
            self.sender.feed_sensor(frame.payload)
        reply = self.assembler.feed(frame)
        if reply is not None:
            with self._msp_cv:
                self._msp_responses[reply["cmd"]] = reply
                self._msp_cv.notify_all()

    def _on_update(self, name: str, values: dict) -> None:
        if name == "device_info":
            self.sender.mark_device_info(values["origin"])
        elif name == "link":
            up = (values.get("up_lq") or 0) > 0
            if up and not self._link_up:
                self.sender.rearm_pings()              # RX/FC can answer now
            self._link_up = up
        if self.log:
            self.log.write(name, values)
        if self.on_telemetry:
            self.on_telemetry(name, values)

    # --- lifecycle ---
    def start(self) -> None:
        self.link.open()
        self.reader.start()
        if not self.listen_only:
            self.sender.start()

    def stop(self) -> None:
        if not self.listen_only:
            self.sender.stop()
        self.reader.stop()
        self.link.close()
        if self.log:
            self.log.close()

    def __enter__(self) -> "Session":
        self.start()
        return self

    def __exit__(self, *exc) -> None:
        self.stop()

    # --- control and commands ---
    def set_controls(self, controls: Controls) -> None:
        self.sender.set_controls(controls)

    def send_frame(self, frame: bytes) -> None:
        self.link.write(frame)

    def bind(self, phrase: Optional[str] = None, uid: Optional[bytes] = None) -> None:
        """Optionally set the bind UID (from a phrase or 6 raw bytes), then send the receiver bind command."""
        if phrase is not None:
            uid = commands.uid_from_phrase(phrase)
        if uid is not None:
            self.send_frame(commands.encode_set_bind_uid(uid))
        self.send_frame(commands.encode_receiver_bind())

    def request_msp(self, cmd: int, timeout: float = 2.0) -> Optional[dict]:
        """Send an MSP-over-CRSF request and wait for the reassembled response (None on timeout)."""
        with self._msp_cv:
            self._msp_responses.pop(cmd, None)
        self.send_frame(commands.encode_msp_request(cmd))
        end = time.monotonic() + timeout
        with self._msp_cv:
            while cmd not in self._msp_responses:
                left = end - time.monotonic()
                if left <= 0:
                    return None
                self._msp_cv.wait(left)
            return self._msp_responses.pop(cmd)

    def take_msp_reply(self, cmd: int) -> Optional[dict]:
        """The newest reply to `cmd` if one has arrived (non-blocking; for the GUI thread), removed from the store."""
        with self._msp_cv:
            return self._msp_responses.pop(cmd, None)

    @property
    def telemetry(self) -> dict:
        return dict(self.reader.latest)
