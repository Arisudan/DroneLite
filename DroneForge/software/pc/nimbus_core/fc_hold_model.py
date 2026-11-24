"""Simulation model of the FC-side height hold (firmware flight/height_hold.c wrapped around the real control law height_hold_core.c, loaded as a shared library built from the
firmware source by tools_scripts/fc_hold_sim/build.sh). SimSession uses it so the PC bring-up script and its abort rules can be tested end to end without hardware.
It mirrors height_hold.c's engage / release rules (the C file itself needs the Betaflight headers and cannot be compiled on the PC); the control law is the firmware's own code."""
from __future__ import annotations

import ctypes
import os
from typing import Optional

LIB_CANDIDATES = (os.environ.get("FC_HOLD_LIB", ""), os.path.expanduser("~/Music/DroneForge/tools_scripts/fc_hold_sim/libhh.so"))
HERE_NAMES = ['kp_z', 'kv_z', 'ki_z', 'i_max', 'vz_lim', 'climb_peak_frac', 'descent_peak_frac', 'thr_idle', 'thr_max', 'drop_max', 'overshoot_m', 'overshoot_vz', 'overshoot_cut', 'height_min', 'height_max',
              'base_tau', 'base_follow_max', 'lift_vz', 'lift_m', 'takeoff_start_frac', 'takeoff_creep_per_s', 'takeoff_cap_frac', 'takeoff_fail_s', 'land_rate', 'land_slow_rate', 'land_slow_below',
              'touch_m', 'touch_s', 'idle_ramp_per_s', 'takeoff_ramp_per_s', 'cap_land_s', 'lift_base_boost', 'takeoff_vz_max', 'takeoff_vz_back', 'takeoff_handover_margin', 'takeoff_fast_per_s']
US_OFF, US_TARGET_START, US_LAND = 1100.0, 1150.0, 1960.0
HOLD_BITS_ACTIVE, HOLD_BITS_FAULT = 0x04, 0x08


def lib_path() -> Optional[str]:
    return next((p for p in LIB_CANDIDATES if p and os.path.exists(p)), None)


def _us(ticks: int) -> float:
    return 5.0 * ticks / 8.0 + 880.0


def _frac(us: float) -> float:
    return min(max(((us - 880.0) * 8.0 / 5.0 - 172.0) / 1639.0, 0.0), 1.0)


class FcHoldModel:
    def __init__(self, thr_max: float = 0.85, **params) -> None:
        path = lib_path()
        if path is None:
            raise RuntimeError("libhh.so not built: run tools_scripts/fc_hold_sim/build.sh")
        L = ctypes.CDLL(path)
        for n in ("hh_update", "hh_integ", "hh_base", "hh_zref"):
            getattr(L, n).restype = ctypes.c_float
        L.hh_params.restype = ctypes.POINTER(ctypes.c_float)
        for n, a in (("hh_engage", 3), ("hh_takeoff", 3), ("hh_set_target", 1), ("hh_follow", 3), ("hh_update", 4)):
            getattr(L, n).argtypes = [ctypes.c_float] * a
        L.xy_update.argtypes = [ctypes.c_float] * 4 + [ctypes.c_int, ctypes.POINTER(ctypes.c_float)]
        L.xy_sign.restype = ctypes.c_float
        L.hh_defaults(); L.xy_defaults(); self.L = L; self.p = L.hh_params()
        self.xy_roll_deg = 0.0; self.xy_pitch_deg = 0.0; self._xy_buf = (ctypes.c_float * 2)()
        self.alt_only = False; self._xy_paused = False          # AUX7 flight mode (firmware v4.2): ALT HOLD pauses the sideways loop
        self.set(thr_max=thr_max, **params)
        self.active = False; self.fault = False; self.latch = False; self.ground_n = 0; self.ground = 0.0; self.out = 0.0

    def set(self, **kw) -> None:
        for k, v in kw.items():
            self.p[HERE_NAMES.index(k)] = v

    def hold_bits(self) -> int:
        mode = self.L.hh_mode() if self.active else 0
        return (HOLD_BITS_ACTIVE if self.active else 0) | (HOLD_BITS_FAULT if self.fault else 0) | ((mode & 3) << 4)

    def set_mode_channel(self, aux7: int) -> None:
        """The firmware's rule: 1100..1399 us on AUX7 = ALT HOLD, anything else (minimum, idle 1500 us, no signal) = POS HOLD."""
        self.alt_only = 1100 <= _us(aux7) < 1400

    def xy_step(self, dt: float, vf: float, vr: float, z: float) -> None:
        if self.alt_only:
            self._xy_paused = True; self.xy_roll_deg = self.xy_pitch_deg = 0.0; return
        if self._xy_paused:
            self.L.xy_reposition(); self._xy_paused = False
        """The FC sideways loop (flight/xy_hold_core.c): body-frame speed in, roll/pitch angle offsets in degrees out (xy_roll_deg, xy_pitch_deg)."""
        if not self.active:
            self.L.xy_defaults(); self.xy_roll_deg = self.xy_pitch_deg = 0.0; return
        mode = self.L.hh_mode()
        self.L.xy_update(ctypes.c_float(dt), ctypes.c_float(vf), ctypes.c_float(vr), ctypes.c_float(self.L.hh_correct(ctypes.c_float(z)) if hasattr(self.L, 'hh_correct') else z), 1 if mode in (0, 1) else 0, self._xy_buf)
        self.xy_roll_deg, self.xy_pitch_deg = float(self._xy_buf[0]), float(self._xy_buf[1])

    def step(self, dt: float, z: float, vz: float, vbat: float, armed: bool, aux5: int, aux6: int, thr_stick: float, level: bool = True) -> Optional[float]:
        """One 50 Hz FC step. Returns the throttle fraction the hold wants, or None while it is not active (the stick throttle is used)."""
        L = self.L
        if not armed:
            self.active = False; self.ground_n = 0; self.ground = 0.0; self.fault = False; self.latch = False
            return None
        ch_us = _us(aux5)
        requested = ch_us >= US_OFF
        land_cmd = requested and ch_us >= US_LAND
        self.latch = self.latch and requested and land_cmd          # as height_hold.c: the landed latch lasts only while the land command is still sent
        if not requested:
            self.fault = False
            if self.active: self.active = False
        stick_us = 880.0 + 5.0 * (172 + thr_stick * 1639.0) / 8.0
        if (not self.active) and self.ground_n < 200 and stick_us < 1300.0 and abs(vz) < 0.15 and z < 0.5:
            self.ground_n += 1; self.ground += (z - self.ground) / self.ground_n
        zr = z - self.ground
        if not self.active:
            ref_ok = aux6 > 0 and _frac(_us(aux6)) > 0.2
            if not (requested and not self.fault and not self.latch and not land_cmd and self.ground_n >= 20):
                return None
            if zr < 0.15:
                if ch_us < US_TARGET_START or not ref_ok or stick_us >= 1300.0 or abs(vz) > 0.15 or not level:
                    return None
                L.hh_takeoff(ctypes.c_float(_frac(_us(aux6))), ctypes.c_float((ch_us - US_TARGET_START) * 2.5 / 800.0), ctypes.c_float(vbat))
            else:
                if abs(vz) > 0.5:
                    return None
                L.hh_engage(ctypes.c_float(zr), ctypes.c_float(_frac(_us(aux6)) if ref_ok else thr_stick), ctypes.c_float(vbat))
            self.active = True
        if zr > 2.8 or zr < -0.2:
            self.active = False; self.fault = True; return None
        if land_cmd:
            L.hh_land()
        elif ch_us >= US_TARGET_START:
            L.hh_set_target(ctypes.c_float((ch_us - US_TARGET_START) * 2.5 / 800.0))
        f = float(L.hh_update(ctypes.c_float(min(max(dt, 0.005), 0.1)), ctypes.c_float(zr), ctypes.c_float(vz), ctypes.c_float(vbat)))
        if L.hh_fail():
            self.active = False; self.fault = True; return None
        if L.hh_landed() and f <= 0.051:
            self.active = False; self.latch = True; return None
        self.out = f
        return f
