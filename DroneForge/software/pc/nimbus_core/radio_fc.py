"""Read-only FC probe over the RADIO link (MSP-over-CRSF): motor outputs (MSP 104), the 16 RC channels as the FC receives them (MSP_RC 105, 2026-11-25) and arming-disable flags (MSP 150), no USB cable needed.

Runs in its OWN thread: a request waits for its reply, and if that blocked the control loop the stale-input failsafe would trip.
At most one request per `period_s` (default 0.5 s), so it costs almost no telemetry bandwidth. Only whitelisted read-only commands can be sent."""
from __future__ import annotations

import struct
import threading
import time
from collections import deque
from typing import Optional

READ_ONLY = (104, 105, 150)
RC_TIMEOUT_S = 1.2          # an MSP reply over ELRS can take 0.7 s: wait longer for the channel read-back than for the others
ARMING_NAMES = ["NOGYRO", "FAILSAFE", "RXLOSS", "BADRX", "BOXFAILSAFE", "RUNAWAY", "CRASH", "THROTTLE", "ANGLE", "BOOTGRACE", "NOPREARM", "LOAD", "CALIB", "CLI",
                "CMS", "BST", "MSP", "PARALYZE", "GPS", "RESCUE_SW", "RPMFILTER", "REBOOT_REQD", "DSHOT_BBANG", "NO_ACC_CAL", "MOTOR_PROTO", "ARMSWITCH"]


def decode_motors(payload: bytes) -> Optional[list]:
    return list(struct.unpack_from("<4H", payload, 0)) if len(payload) >= 8 else None


def decode_rc(payload: bytes) -> Optional[list]:
    """MSP_RC: one little-endian uint16 per channel the receiver delivers (microseconds as the FC sees them, rcData[0..n-1]); None when shorter than 4 channels."""
    n = len(payload) // 2
    if n < 4: return None
    v = list(struct.unpack_from("<%dH" % min(n, 18), payload, 0))
    v[2], v[3] = v[3], v[2]          # Betaflight's rcData order is roll, pitch, YAW, THROTTLE; the GCS / CRSF order is roll, pitch, THROTTLE, yaw (seen live 2026-11-25: CH3 +512 / CH4 -512)
    return v


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
        self.rc: Optional[list] = None          # the FC's rcData (us), newest reply
        self.t_rc: Optional[float] = None       # when that reply arrived
        self.rc_req_t: Optional[float] = None   # when its REQUEST was sent (the FC read its channels about then): a test pattern counts only if it was applied before this
        self.rc_rtt_s: Optional[float] = None
        self.rc_requests = self.rc_replies = 0
        self._rc_times: deque = deque(maxlen=20)
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    @property
    def rc_reply_hz(self) -> float:
        """How often the FC's channel read-back arrives (replies per second over the last ~10 s)."""
        t = list(self._rc_times)
        return (len(t) - 1) / (t[-1] - t[0]) if len(t) >= 2 and t[-1] > t[0] else 0.0

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
        if cmd == 105: self.rc_requests += 1
        t_req = time.monotonic()
        r = self.sess.request_msp(cmd, timeout=RC_TIMEOUT_S if cmd == 105 else self.timeout)
        if r:
            self.replies += 1
            if cmd == 105: self._t_req_last = t_req
        return r

    def _run(self) -> None:
        n = 0
        while not self._stop.is_set():
            try:
                r = self._ask((105, 104, 105, 150)[n % 4])
            except OSError:                       # the radio dongle went away (unplugged, reset, or the GCS is closing): stop quietly, the link watchdog reports it
                self._stop.set(); break
            if r:
                if r["cmd"] == 104:
                    m = decode_motors(r["payload"])
                    if m:
                        self.motors, self.t_motors = m, time.monotonic()
                elif r["cmd"] == 105:
                    rc = decode_rc(r["payload"])
                    if rc:
                        now = time.monotonic(); self.rc, self.t_rc, self.rc_req_t = rc, now, getattr(self, "_t_req_last", now); self.rc_rtt_s = now - self.rc_req_t
                        self.rc_replies += 1; self._rc_times.append(now)
                else:
                    b = decode_blockers(r["payload"])
                    if b is not None:
                        self.blockers = b
            n += 1
            self._stop.wait(self.period)
