"""Read-only FC probe over the RADIO link (MSP-over-CRSF): motor outputs (MSP 104) and arming-disable flags (MSP 150), no USB cable needed.

Runs in its OWN thread: a request waits for its reply, and if that blocked the control loop the stale-input failsafe would trip.
At most one request per `period_s` (default 0.5 s), so it costs almost no telemetry bandwidth. Only whitelisted read-only commands can be sent."""
from __future__ import annotations

import struct
import threading
import time
from typing import Optional

READ_ONLY = (104, 150)
ARMING_NAMES = ["NOGYRO", "FAILSAFE", "RXLOSS", "BADRX", "BOXFAILSAFE", "RUNAWAY", "CRASH", "THROTTLE", "ANGLE", "BOOTGRACE", "NOPREARM", "LOAD", "CALIB", "CLI",
                "CMS", "BST", "MSP", "PARALYZE", "GPS", "RESCUE_SW", "RPMFILTER", "REBOOT_REQD", "DSHOT_BBANG", "NO_ACC_CAL", "MOTOR_PROTO", "ARMSWITCH"]


def decode_motors(payload: bytes) -> Optional[list]:
    return list(struct.unpack_from("<4H", payload, 0)) if len(payload) >= 8 else None


def decode_blockers(payload: bytes) -> Optional[list]:
    if len(payload) < 22:
        return None
    idx = 16 + payload[15]
    if len(payload) < idx + 5:
        return None
    flags = struct.unpack_from("<I", payload, idx + 1)[0]
    return [ARMING_NAMES[i] for i in range(len(ARMING_NAMES)) if flags >> i & 1]


class RadioFcProbe:
    def __init__(self, session, period_s: float = 0.5, timeout_s: float = 0.3) -> None:
        self.sess, self.period, self.timeout = session, period_s, timeout_s
        self.motors: Optional[list] = None
        self.t_motors: Optional[float] = None
        self.blockers: Optional[list] = None
        self.replies = self.requests = 0
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="radio-fc-probe", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.5)

    def _ask(self, cmd: int):
        assert cmd in READ_ONLY, "radio probe only sends read-only MSP requests"
        self.requests += 1
        r = self.sess.request_msp(cmd, timeout=self.timeout)
        if r:
            self.replies += 1
        return r

    def _run(self) -> None:
        n = 0
        while not self._stop.is_set():
            try:
                r = self._ask(104 if n % 2 == 0 else 150)
            except OSError:                       # the radio dongle went away (unplugged, reset, or the GCS is closing): stop quietly, the link watchdog reports it
                self._stop.set(); break
            if r:
                if r["cmd"] == 104:
                    m = decode_motors(r["payload"])
                    if m:
                        self.motors, self.t_motors = m, time.monotonic()
                else:
                    b = decode_blockers(r["payload"])
                    if b is not None:
                        self.blockers = b
            n += 1
            self._stop.wait(self.period)
