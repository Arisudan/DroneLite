"""Qt-free sensor calibration helpers. Calibration itself happens ON THE FC; the PC only commands it and shows the result.

LEVEL (accelerometer): while the FC is DISARMED the Betaflight stick gesture THROTTLE FULL + YAW LEFT + PITCH BACK is sent for 2.5 s over the radio
(same method as tools_scripts/fc_config/fc_calibrate_accel_level_over_radio.py). The arm channel stays LOW the whole time: this never arms and the motors do not spin.
The FC measures for ~0.5 s and saves by itself; the rest tilt is read before and after. Controls are ALWAYS restored to idle in a finally.
GYRO: the FC calibrates the gyro by itself at every arm; here only a rest check (offset and noise of the gyro rate from telemetry) is offered.
"""
from __future__ import annotations

import math
import statistics
import threading
import time
from collections import deque
from typing import Optional

from nimbus_core import topics as T
from nimbus_pc.sender import Controls

LEVEL_GESTURE_S = 2.5           # throttle full + yaw left + pitch back, arm low
LEVEL_SETTLE_S = 3.0            # sticks centred: the FC measures (~0.5 s) and saves
LEVEL_AFTER_S = 2.0             # then the rest tilt is read again over this long
LEVEL_OK_DEG = 0.3              # success: both |after| <= this
STILL_WINDOW_S, STILL_MAX_DEG = 2.0, 0.3
MIN_LQ = 50
FRESH_S = 1.5                   # attitude (ATTITUDE frame ~1.2 Hz), link and battery must be newer than this
STATUS_FRESH_S = 3.0            # flight-mode frames carry the armed state (diagnostic groups rotate through them)
GYRO_CHECK_S, GYRO_MIN_SAMPLES = 3.0, 8
GYRO_OFFSET_DPS, GYRO_NOISE_DPS = 1.0, 0.5       # above these: NEEDS RE-CAL; above half of them: CAUTION
GREEN_DEG, AMBER_DEG = 0.5, 1.5
NOT_LEVEL = "NOT level: check the surface is level"


def tilt_colour(deg: Optional[float]) -> str:
    if deg is None:
        return "grey"
    d = abs(deg)
    return "green" if d < GREEN_DEG else "amber" if d < AMBER_DEG else "red"


def gyro_verdict(offset_dps: float, noise_dps: float) -> str:
    if offset_dps > GYRO_OFFSET_DPS or noise_dps > GYRO_NOISE_DPS:
        return "NEEDS RE-CAL"
    if offset_dps > GYRO_OFFSET_DPS / 2 or noise_dps > GYRO_NOISE_DPS / 2:
        return "CAUTION"
    return "OK"


class Calibrator:
    def __init__(self, core, clock=time.monotonic, sleep=time.sleep) -> None:
        self.core, self._clock, self._sleep = core, clock, sleep
        self._thread: Optional[threading.Thread] = None
        self.running = False
        self.progress = ""                            # one line while the level calibration runs
        self.level_result: Optional[dict] = None      # {'before': (pitch, roll), 'after': ..., 'ok': bool, 'text': str}
        self.last_controls = Controls()
        self._lock = threading.Lock()
        self._att: deque = deque(maxlen=400)          # (t, pitch_deg, roll_deg)
        self._gyro: deque = deque(maxlen=2000)        # (t, r, p, y) deg/s
        self._t_status = self._t_link = self._t_batt = None
        self._armed_text: Optional[bool] = None
        self._lq: Optional[float] = None
        self._batt_v: Optional[float] = None
        self._gyro_t0: Optional[float] = None
        self.gyro_result: Optional[dict] = None
        bus = getattr(core, "bus", None)
        if bus is not None:
            bus.subscribe(T.TELEMETRY, lambda m: self.observe(m[0], m[1]))

    # --- telemetry in ---------------------------------------------------------------------------
    def observe(self, name: str, d: dict, now: Optional[float] = None) -> None:
        now = self._clock() if now is None else now
        with self._lock:
            if name == "attitude":
                self._att.append((now, math.degrees(d["pitch_rad"]), math.degrees(d["roll_rad"])))
            elif name == "flight_mode":
                self._t_status, self._armed_text = now, not str(d.get("mode", "")).endswith("*")
                if not d.get("mode"):
                    self._t_status = None
                if "gyro_dps10" in d:
                    self._gyro.append((now, *(v / 10.0 for v in d["gyro_dps10"])))
            elif name == "link":
                self._t_link, self._lq = now, d.get("up_lq")
            elif name == "battery":
                self._t_batt, self._batt_v = now, d.get("voltage_v")

    def _window(self, secs: float, since: float = -1e18) -> list:
        now = self._clock()
        with self._lock:
            return [s for s in self._att if now - s[0] <= secs and s[0] >= since]

    @staticmethod
    def _age(t: Optional[float], now: float) -> Optional[float]:
        return None if t is None else now - t

    def attitude_age(self) -> Optional[float]:
        """Age of the freshest attitude information (either source, see attitude_select), else of the last sample we saw."""
        now, t = self._clock(), None
        sel = getattr(self.core, "att_sel", None)
        try:
            t = sel.last_time() if sel is not None else None
        except Exception:                                   # noqa: BLE001
            t = None
        with self._lock:
            t2 = self._att[-1][0] if self._att else None
        ts = [x for x in (t, t2) if x is not None]
        return None if not ts else now - max(ts)

    # --- level status ---------------------------------------------------------------------------
    def level_status(self) -> dict:
        """Rest tilt in degrees from the freshest attitude: {'pitch','roll','colour','age','stale'}; pitch/roll None = no usable data."""
        with self._lock:
            last = self._att[-1] if self._att else None
        age = self.attitude_age()
        if last is None:
            att = None
            try:
                att = self.core.state.decoded.get("attitude")
            except AttributeError:
                pass
            last = (0, math.degrees(att["pitch_rad"]), math.degrees(att["roll_rad"])) if att else None
        if last is None:
            return {"pitch": None, "roll": None, "colour": "grey", "age": age, "stale": True}
        stale = age is None or age > FRESH_S
        p, r = last[1], last[2]
        return {"pitch": p, "roll": r, "colour": "grey" if stale else tilt_colour(max(abs(p), abs(r))), "age": age, "stale": stale}

    # --- pre-checks (enforced here, not only in the UI) -----------------------------------------
    def _common_refusal(self) -> Optional[str]:
        c, now = self.core, self._clock()
        if not getattr(c, "connected", False):
            return "Not connected: connect to the drone first."
        if getattr(c, "state_source", "live") in ("sim", "bench"):
            return "Not available in the simulator or bench mode: there is no real FC."
        if self.running:
            return "A calibration is already running."
        au = getattr(c, "autonomy", None)
        if self._t_status is None or now - self._t_status > STATUS_FRESH_S:
            return "No fresh FC status: wait for the telemetry (is the FC powered by the battery?)."
        if self._armed_text or getattr(au, "fc_armed", False):
            return "The FC is armed: disarm first."
        if au is not None and getattr(au, "phase", "idle") != "idle":
            return "A flight or test is running: stop it first."
        if getattr(getattr(c, "ramp", None), "phase", "idle") not in ("idle", "done"):
            return "A throttle ramp is running: stop it first."
        if getattr(c, "source", "idle") not in ("idle", "external"):
            return "Controls are in use by another source: set the source to idle first."
        return None

    def level_refusal(self) -> Optional[str]:
        why = self._common_refusal()
        if why:
            return why
        now = self._clock()
        if self._t_link is None or now - self._t_link > FRESH_S or self._lq is None:
            return "No fresh radio link data."
        if self._lq < MIN_LQ:
            return f"Link quality is {self._lq:.0f} %: needs at least {MIN_LQ} %."
        age = self.attitude_age()
        if age is None or age > FRESH_S:
            return "No fresh attitude data from the FC."
        if self._t_batt is None or now - self._t_batt > 5.0 or self._batt_v is None:
            return "No battery voltage: connect the flight battery (the FC must run on it)."
        w = self._window(STILL_WINDOW_S)
        if len(w) < 2:
            return "Not enough attitude data yet to check the drone is still: wait a moment."
        span = max(max(s[1] for s in w) - min(s[1] for s in w), max(s[2] for s in w) - min(s[2] for s in w))
        if span > STILL_MAX_DEG:
            return f"The drone is not still (attitude moves {span:.1f} deg): keep it completely still."
        return None

    # --- level calibration ----------------------------------------------------------------------
    def calibrate_level(self) -> tuple[bool, str]:
        why = self.level_refusal()
        if why:
            return False, why
        w = self._window(STILL_WINDOW_S)
        before = (statistics.median(s[1] for s in w), statistics.median(s[2] for s in w))
        self.running, self.level_result = True, None
        self.progress = "Sending the calibration command (2.5 s), do not touch the drone"
        self._prev_source = getattr(self.core, "source", "idle")
        self._thread = threading.Thread(target=self._run_level, args=(before,), daemon=True)
        self._thread.start()
        return True, self.progress

    def _hold(self, ctl: Controls, secs: float) -> Optional[str]:
        """Send ctl for secs through the core; returns a reason when it must stop (SAFE pressed, disarm lost, armed, disconnected)."""
        c, t0 = self.core, self._clock()
        while self._clock() - t0 < secs:
            if not getattr(c, "connected", False):
                return "the link dropped"
            if getattr(c, "source", "") != "external":
                return "SAFE or another source took over the controls"
            if self._armed_text or getattr(getattr(c, "autonomy", None), "fc_armed", False):
                return "the FC armed"
            self.last_controls = ctl
            c.set_external_controls(ctl)
            self._sleep(0.02)
        return None

    def _run_level(self, before) -> None:
        c, res = self.core, None
        try:
            if getattr(c, "source", "idle") != "external":
                c.set_source("external")
            stop = self._hold(Controls(roll=0.0, pitch=-1.0, yaw=-1.0, throttle=1.0, arm=False), LEVEL_GESTURE_S)
            if not stop:
                self.progress = "FC is measuring and saving (3 s), keep the drone still"
                stop = self._hold(Controls(), LEVEL_SETTLE_S)
            if not stop:
                self.progress = "Reading the rest tilt again"
                t0 = self._clock()
                stop = self._hold(Controls(), LEVEL_AFTER_S)
                w = self._window(LEVEL_AFTER_S + 0.5, since=t0)
                if not stop and not w:
                    stop = "no attitude data came back"
                if not stop:
                    after = (statistics.median(s[1] for s in w), statistics.median(s[2] for s in w))
                    ok = abs(after[0]) <= LEVEL_OK_DEG and abs(after[1]) <= LEVEL_OK_DEG
                    txt = (f"Level calibrated. Before: pitch {before[0]:+.1f} deg, roll {before[1]:+.1f} deg. "
                           f"After: pitch {after[0]:+.1f} deg, roll {after[1]:+.1f} deg." if ok else
                           f"{NOT_LEVEL}. Before: pitch {before[0]:+.1f} deg, roll {before[1]:+.1f} deg. "
                           f"After: pitch {after[0]:+.1f} deg, roll {after[1]:+.1f} deg. The calibration was still done.")
                    res = {"before": before, "after": after, "ok": ok, "text": txt}
            if stop:
                res = {"before": before, "after": None, "ok": False, "text": f"Level calibration stopped: {stop}. Nothing was verified."}
        except Exception as e:                                  # noqa: BLE001
            res = {"before": before, "after": None, "ok": False, "text": f"Level calibration failed: {e}"}
        finally:
            self.last_controls = Controls()
            try:
                c.set_external_controls(Controls())
                if getattr(self, "_prev_source", "idle") != "external" and getattr(c, "source", "") == "external":
                    c.set_source(self._prev_source)
            except Exception:                                   # noqa: BLE001
                pass
            self.level_result, self.progress, self.running = res, "", False

    # --- gyro rest check ------------------------------------------------------------------------
    def gyro_refusal(self) -> Optional[str]:
        why = self._common_refusal()
        if why:
            return why
        if self._gyro_t0 is not None:
            return "The gyro check is already running."
        return None

    def start_gyro_check(self) -> tuple[bool, str]:
        why = self.gyro_refusal()
        if why:
            return False, why
        self._gyro_t0, self.gyro_result = self._clock(), None
        return True, "Measuring the gyro at rest for 3 s: keep the drone still"

    def gyro_poll(self) -> Optional[dict]:
        """Finishes the check once 3 s have passed (call from a timer); None while measuring / idle. Result: {'verdict','offset','noise','n','text'}."""
        if self._gyro_t0 is None:
            return self.gyro_result
        now = self._clock()
        if self._armed_text:
            self._gyro_t0, self.gyro_result = None, {"verdict": "", "n": 0, "text": "The FC armed: the gyro check was cancelled."}
            return self.gyro_result
        if now - self._gyro_t0 < GYRO_CHECK_S:
            return None
        t0, self._gyro_t0 = self._gyro_t0, None
        with self._lock:
            s = [x for x in self._gyro if x[0] >= t0]
        if len(s) < GYRO_MIN_SAMPLES:
            self.gyro_result = {"verdict": "", "n": len(s), "text": "No gyro data from the FC: its telemetry does not carry the gyro rate "
                                f"(only {len(s)} samples in 3 s). The gyro cannot be checked from here."}
            return self.gyro_result
        axes = list(zip(*[x[1:] for x in s]))
        offset = max(abs(statistics.fmean(a)) for a in axes)
        noise = max(statistics.pstdev(a) for a in axes)
        v = gyro_verdict(offset, noise)
        self.gyro_result = {"verdict": v, "offset": offset, "noise": noise, "n": len(s),
                            "text": f"{v}: offset {offset:.2f} deg/s, noise {noise:.2f} deg/s ({len(s)} samples)"}
        return self.gyro_result

    def gyro_measuring(self) -> bool:
        return self._gyro_t0 is not None
