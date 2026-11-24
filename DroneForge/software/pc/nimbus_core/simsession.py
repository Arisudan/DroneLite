"""Sim state source (Nimbus: StateSourceType.Sim): a Session-compatible backend driven by the physics simulator, so the GCS, the safety layer
and the hover autonomy can be used with NO hardware. It looks like nimbus_pc.session.Session to NimbusCore: same callbacks
(on_telemetry with decoded dicts, on_frame with Frames), same sender attributes the GCS status reads.

Plant = nimbus_pc.hover.sim.DroneSim (ANGLE-mode response, motor lag, battery sag, sensor noise/delay/drops). The 'FC' arms when it is
given arm HIGH at near-zero throttle, like the real one (it ignores arm requested with throttle high)."""
from __future__ import annotations

import threading
import time
from dataclasses import replace
from typing import Callable, Optional

from nimbus_pc.crsf import ADDR_FC, TYPE_ATTITUDE, TYPE_BATTERY, TYPE_FLIGHT_MODE, TYPE_LINK_STATISTICS, TYPE_OPTRANGE, Frame
from nimbus_pc.hover.sim import DroneSim, SimParams
from nimbus_pc.sender import Controls


class _Link:
    on_write: Optional[Callable] = None


class _Parser:
    crc_errors = 0


class _Reader:
    parser = _Parser()


class _Sender:
    def __init__(self) -> None:
        self.allow_arm = False
        self.failsafe_active = False
        self.frames_sent = 0
        self.pings_sent = 0
        self.max_interval_s = 0.0
        self.alive = True


class SimSession:
    """Drop-in for Session in NimbusCore (sim=True). Thread at 100 Hz steps the plant with the latest controls."""

    def __init__(self, params: Optional[SimParams] = None, seed: int = 1, on_telemetry: Optional[Callable] = None,
                 on_frame: Optional[Callable] = None, realtime: bool = True, ctrl_delay_s: float = 0.022, **_ignored) -> None:
        self.params = replace(params or SimParams(), ctrl_delay_s=ctrl_delay_s)
        self.sim = DroneSim(self.params, seed)
        self.on_telemetry, self.on_frame = on_telemetry, on_frame
        self.link, self.sender, self.reader, self.watchdog = _Link(), _Sender(), _Reader(), None
        self.realtime = realtime
        self._controls = Controls()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self.fc_armed = False
        self.fc_hold = None                     # FcHoldModel (the firmware's height-hold control law) when the sim is asked to model the FC-side hold
        self._n_step = 0
        self._fc_thr = None
        self._next_link = 0.0
        self._next_mode = 0.0
        self.t0 = time.monotonic()
        self.dt = 0.01

    # --- Session interface -------------------------------------------------------------------
    def start(self) -> None:
        for origin, name in ((0xEE, "Simulated TX"), (0xEC, "Simulated RX"), (0xC8, "Simulated FC (Betaflight)")):
            if self.on_telemetry:
                self.on_telemetry("device_info", {"origin": origin, "dest": 0xEA, "name": name})
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="sim-session", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)

    def set_controls(self, c: Controls) -> None:
        with self._lock:
            self._controls = c

    # --- plant loop --------------------------------------------------------------------------
    def _arm_logic(self, c: Controls) -> bool:
        want = bool(c.arm and self.sender.allow_arm)
        if not want:
            self.fc_armed = False
        elif not self.fc_armed and c.throttle <= 0.06:       # arms only with the throttle low, like Betaflight
            self.fc_armed = True
        return self.fc_armed

    def step(self) -> None:
        with self._lock:
            c = self._controls
        armed = self._arm_logic(c)
        sim = self.sim
        thr = c.throttle
        if self.fc_hold is not None:
            self._n_step += 1
            if self._n_step % 2 == 0:                                         # the FC hold runs at 50 Hz on the FC's own height and speed (no radio in this loop)
                self._fc_thr = self.fc_hold.step(0.02, sim.z, sim.vz, sim.batt_v, armed, c.aux5, c.aux6, c.throttle)
            if self._fc_thr is not None and armed:
                thr = self._fc_thr
        roll_c, pitch_c = c.roll, c.pitch
        if self.fc_hold is not None:
            if self._n_step % 2 == 0:
                self.fc_hold.set_mode_channel(c.aux7)
                self.fc_hold.xy_step(0.02, sim.vx, sim.vy, sim.z)       # the sim's world frame has yaw 0: forward = x, right = y
            lim = max(self.params.angle_limit_deg, 1.0)
            roll_c += self.fc_hold.xy_roll_deg / lim * self.params.roll_sign; pitch_c += self.fc_hold.xy_pitch_deg / lim * self.params.pitch_sign
        sim.step(self.dt, roll_c, pitch_c, thr, armed)
        self.sender.frames_sent += 1
        if self.link.on_write:
            self.link.on_write(b"\xee\x1a\x16" + bytes(24))
        self._emit_due()

    def _frame(self, ftype: int) -> None:
        if self.on_frame:
            self.on_frame(Frame(ADDR_FC, ftype, b""))

    def _emit(self, ftype: int, name: str, data: dict) -> None:
        self._frame(ftype)
        if self.on_telemetry:
            self.on_telemetry(name, data)

    def _emit_due(self) -> None:
        sim = self.sim
        for smp in sim.poll():
            d = smp.data
            if smp.kind == "optrange":
                dist = d["distance_m"]
                self._emit(TYPE_OPTRANGE, "optrange", {"distance_m": dist, "dist_strength": 150, "dist_precision": 1,
                                                        "velocity_forward": d["flow_vx"], "velocity_right": d["flow_vy"],
                                                        "flow_quality": d["quality"], "flow_status": 1,
                                                        "flow_x_raw": 0, "flow_y_raw": 0})
                if self.fc_hold is not None:                                   # the compact state frame's EKF half, with the FC hold status bits
                    bits = self.fc_hold.hold_bits()
                    self._emit(0xD4, "kinematic", {"x_m": float("nan"), "y_m": float("nan"), "z_m": float("nan"), "vx_mps": sim.vx, "vy_mps": sim.vy, "vz_mps": sim.vz,
                                                   "roll_rad": sim.tilt_y, "pitch_rad": sim.tilt_x, "yaw_rad": 0.0, "valid": True,
                                                   "hold_active": bool(bits & 4), "hold_fault": bool(bits & 8), "hold_mode": (bits >> 4) & 3, "vel_body": True})
            elif smp.kind == "attitude":
                import math
                self._emit(TYPE_ATTITUDE, "attitude", {"pitch_rad": math.radians(d["pitch_deg"]), "roll_rad": math.radians(d["roll_deg"]), "yaw_rad": 0.0})
            elif smp.kind == "battery":
                v = d["voltage_v"]
                self._emit(TYPE_BATTERY, "battery", {"voltage_v": v, "current_a": 0.0, "capacity_mah": 0, "remaining_pct": int(max(0, min(100, (v - 13.2) / (16.8 - 13.2) * 100)))})
        t = sim.t
        if t >= self._next_link:
            self._next_link = t + 0.25
            self._emit(TYPE_LINK_STATISTICS, "link", {"up_rssi1_dbm": -45, "up_rssi2_dbm": -48, "up_lq": 100, "up_snr_db": 12, "antenna": 0, "rf_mode": 4,
                                                      "tx_power_idx": 1, "down_rssi_dbm": -45, "down_lq": 100, "down_snr_db": 12})
        if t >= self._next_mode:
            self._next_mode = t + 0.15
            self._emit(TYPE_FLIGHT_MODE, "flight_mode", {"mode": "STAB" if self.fc_armed else "STAB*"})

    def _run(self) -> None:
        nxt = time.monotonic()
        while not self._stop.is_set():
            self.step()
            nxt += self.dt
            delay = nxt - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            elif delay < -0.5:
                nxt = time.monotonic()
