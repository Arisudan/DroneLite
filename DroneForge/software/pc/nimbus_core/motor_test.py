"""Hold-to-spin test of ONE motor over the radio (MSP_SET_MOTOR, Betaflight honours it only while DISARMED).

The FC does NOT time out MSP motor values by itself: a stuck value would keep a prop turning. So every path out of a spin goes through stop(), which sends all-1000
three times 100 ms apart and then keeps resending zeros (worker thread) until the MSP_MOTOR readback shows all four motors at idle or WATCHDOG_S has passed.
All safety gates are enforced here, not only in the UI. States: idle / spinning / stopping."""
from __future__ import annotations

import threading
import time
from typing import List, Optional

from nimbus_pc import commands
from . import topics as T
from .radio_fc import decode_motors

MOTOR_TEST_US = 1075            # about 7 %: fixed, low
MAX_TEST_US = 1100              # hard ceiling, enforced again when the frame is built
IDLE_US = 1000
IDLE_TOL_US = 10                # readback at or below this counts as stopped
MAX_HOLD_S = 2.5                # auto-stop per press
MIN_LQ = 50
STATUS_FRESH_S = 1.0
LQ_FRESH_S = 2.0
VERIFY_S = 1.0
WATCHDOG_S = 3.0
ZERO_GAP_S = 0.1
ZEROS_MIN = 3
POLL_S = 0.2


class MotorTest:
    def __init__(self, core) -> None:
        self.core = core
        self.state = "idle"                       # idle | spinning | stopping
        self.motor = 0
        self.value_us = MOTOR_TEST_US
        self.t_start = 0.0
        self.status_text = ""                     # plain words for the UI
        self.unconfirmed = False
        self.armed: Optional[bool] = None; self.t_status = 0.0     # from the flight-mode text ('STAB*' = disarmed)
        self.lq: Optional[int] = None; self.t_lq = 0.0
        self.readback: Optional[List[int]] = None; self.t_readback = 0.0
        self.frames_sent = 0; self.zero_rounds = 0
        self._lock = threading.RLock()
        self._thread: Optional[threading.Thread] = None
        self._stop_reason = ""
        core.bus.subscribe(T.TELEMETRY, self._on_tel)

    # --- inputs ---------------------------------------------------------------------------
    def _on_tel(self, msg) -> None:
        name, d = msg
        now = time.monotonic()
        if name == "flight_mode" and "mode" in d:
            self.armed = not str(d["mode"]).endswith("*"); self.t_status = now
            if "motors" in d:
                self._set_readback(d["motors"])
        elif name == "link" and d.get("up_lq") is not None:
            self.lq, self.t_lq = int(d["up_lq"]), now

    def _set_readback(self, motors) -> None:
        self.readback, self.t_readback = [int(x) for x in motors][:4], time.monotonic()

    # --- gates ----------------------------------------------------------------------------
    def refusal(self) -> str:
        """Plain-words reason why a motor must not spin now, or '' when every gate passes."""
        c = self.core
        if not c.connected or c.sess is None:
            return "Connect to the drone first."
        if getattr(c, "state_source", "live") != "live":
            return "Motor test works only on the live radio link."
        now = time.monotonic()
        if self.armed is None or now - self.t_status > STATUS_FRESH_S:
            return "No fresh FC status (under 1 s old): wait for the link."
        if self.armed:
            return "FC is armed: disarm first."
        if self.lq is None or now - self.t_lq > LQ_FRESH_S:
            return "No link-quality reading yet."
        if self.lq < MIN_LQ:
            return f"Link quality is {self.lq} % (need {MIN_LQ} % or more)."
        if c.ramp.phase in ("arming", "ramping") or c.autonomy.phase != "idle" or c.autonomy.public_phase() not in ("ready", "landed"):
            return "A flight or test is already running."
        return ""

    # --- control --------------------------------------------------------------------------
    def _values(self, motor: int) -> List[int]:
        v = [IDLE_US] * 4
        if motor in (1, 2, 3, 4):
            v[motor - 1] = max(IDLE_US, min(MAX_TEST_US, int(self.value_us)))
        return v

    def _send(self, values: List[int]) -> bool:
        sess = self.core.sess
        if sess is None:
            return False
        try:
            for f in commands.encode_msp_set_motor(values):
                sess.send_frame(f)
            self.frames_sent += 1
            return True
        except Exception:                                        # radio unplugged: the link watchdog reports it
            return False

    def start(self, motor: int) -> str:
        """Begin spinning `motor` (1-4). Returns '' when started, else the plain refusal reason."""
        with self._lock:
            if motor not in (1, 2, 3, 4):
                return "Pick motor 1 to 4."
            if self.state == "stopping":
                return "Still stopping: wait a moment."
            why = self.refusal()
            if why:
                self.status_text = why
                return why
            if self.state == "spinning" and self.motor != motor:
                self.stop("another motor pressed")
                return "Still stopping: wait a moment."
            self.motor, self.unconfirmed = motor, False
            self.t_start = time.monotonic(); self.readback = None
            if not self._send(self._values(motor)):
                self.status_text = "Could not send to the radio."
                return self.status_text
            self.state = "spinning"
            self.status_text = f"M{motor} SPINNING (release to stop)"
            self.core.event(f"MOTOR TEST: M{motor} spin start")
            self._ensure_worker()
            return ""

    def refresh(self) -> None:
        """Called at 10 Hz while the button is held: keep-alive, 2.5 s cap, gate re-check, readback confirmation."""
        with self._lock:
            if self.state != "spinning":
                return
            now = time.monotonic()
            if now - self.t_start > MAX_HOLD_S:
                self.stop(f"{MAX_HOLD_S:g} s limit"); self.status_text = f"M{self.motor} stopped after {MAX_HOLD_S:g} s: press again to continue."; return
            why = self.refusal()
            if why:
                self.stop(why); self.status_text = why; return
            self._send(self._values(self.motor))
            if not self.unconfirmed and now - self.t_start >= VERIFY_S:
                rb = self.readback if self.t_readback > self.t_start else None
                if rb is None or rb[self.motor - 1] <= IDLE_US:
                    self.unconfirmed = True
                    self.status_text = f"FC did not spin M{self.motor} (it may not accept motor commands over the radio): not confirmed"
                    self.core.event(f"MOTOR TEST: M{self.motor} not confirmed by the readback")

    def stop(self, reason: str = "release", blocking: bool = False) -> None:
        """All motors to 1000, three times 100 ms apart, then the worker keeps resending until the readback is idle or WATCHDOG_S. Safe to call anytime."""
        with self._lock:
            if self.state == "idle":
                return
            if self.state == "spinning":
                self.core.event(f"MOTOR TEST: M{self.motor} spin stop ({reason})")
                if not self.status_text.startswith(("FC did not", "M%d stopped" % self.motor)):
                    self.status_text = f"M{self.motor} stopped"
            self.state = "stopping"; self._stop_reason = reason; self.zero_rounds = 0
            self._send([IDLE_US] * 4); self.zero_rounds = 1
            if blocking:
                for _ in range(ZEROS_MIN - 1):
                    time.sleep(ZERO_GAP_S); self._send([IDLE_US] * 4); self.zero_rounds += 1
                self.state = "idle"; return
            self._ensure_worker()

    def _ensure_worker(self) -> None:
        if self._thread is None or not self._thread.is_alive():
            self._thread = threading.Thread(target=self._run, name="motor-test", daemon=True); self._thread.start()

    def wait_idle(self, timeout: float = 5.0) -> bool:
        t = self._thread
        if t is not None:
            t.join(timeout)
        return self.state == "idle"

    def _poll(self) -> None:
        sess = self.core.sess
        if sess is None:
            return
        try:
            r = sess.request_msp(104, timeout=0.3)
        except Exception:
            return
        if r and r.get("cmd") == 104:
            m = decode_motors(r["payload"])
            if m:
                self._set_readback(m)

    def _run(self) -> None:
        t_stop = None; last_zero = 0.0
        while True:
            with self._lock:
                st = self.state
                if st == "idle":
                    return
                if st == "stopping":
                    now = time.monotonic()
                    if t_stop is None:
                        t_stop = now; last_zero = now
                    if now - last_zero >= ZERO_GAP_S - 1e-6:
                        self._send([IDLE_US] * 4); self.zero_rounds += 1; last_zero = now
                    rb_ok = self.readback is not None and self.t_readback > t_stop - POLL_S and max(self.readback) <= IDLE_US + IDLE_TOL_US
                    if (self.zero_rounds >= ZEROS_MIN and rb_ok) or now - t_stop >= WATCHDOG_S:
                        if not rb_ok:
                            self.core.event("MOTOR TEST: readback never showed all motors idle: unplug the battery if a prop still turns")
                            self.status_text = "Stop sent, but the readback did not confirm idle: check the props."
                        self.state = "idle"; return
                else:
                    t_stop = None
            self._poll()
            time.sleep(min(POLL_S, ZERO_GAP_S / 2) if st == "stopping" else POLL_S)
