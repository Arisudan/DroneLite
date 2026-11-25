"""Live hold log (2026-11-25): one CSV row per 0.1 s while the radio is connected (1 Hz while the FC is disarmed), for the question "why did the sideways hold not act?".
Per row: what the GCS SENDS on the hold channels (arm CH5, height CH9, mode CH11, aid CH13-15), what the FC READS BACK on them (MSP_RC, about every 3 s, with its age),
the aid the core holds (m, age), the FC's own velocity / attitude / range from telemetry, the flight phase, and the VO engine status (key frame, fresh aid). Qt-free.
Files: <DroneForge>/docs/status_logs/hold_live/<date>_<time>_hold.csv (never on the SD card, never sent anywhere)."""
from __future__ import annotations

import csv
import math
import os
import time
from typing import Optional

COLS = ["t_s", "source", "phase", "fc_armed", "fc_mode", "explore", "sent_arm", "sent_height_us", "sent_mode_us", "sent_aid_fwd_us", "sent_aid_right_us", "sent_aid_valid_us",
        "core_aid_fwd_m", "core_aid_right_m", "core_aid_age_s", "fc_ch5_us", "fc_height_us", "fc_mode_us", "fc_aid_fwd_us", "fc_aid_right_us", "fc_aid_valid_us", "fc_rc_age_s",
        "fc_vfwd_mps", "fc_vright_mps", "range_m", "pitch_deg", "roll_deg", "vo_anchor", "vo_fresh_aid", "vo_text"]
DEFAULT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "docs", "status_logs", "hold_live")


def _us(ticks) -> Optional[int]:
    return None if ticks is None else int(round(ticks * 5.0 / 8.0 + 880.0))


class HoldLog:
    def __init__(self, core, directory: Optional[str] = None, clock=time.monotonic) -> None:
        self.core, self.dir, self.clock = core, os.path.abspath(directory or os.environ.get("DF_HOLD_LOG_DIR") or DEFAULT_DIR), clock
        self.explicit = directory is not None          # the real log folder is written only by a live radio session (tests, the simulator and the bench do not litter it)
        self.f = None; self.w = None; self.path: Optional[str] = None; self.t0 = 0.0; self._last = 0.0; self.rows = 0; self.last: dict = {}
        from nimbus_core import topics as T
        core.bus.subscribe(T.TELEMETRY, self._on_tel)

    def _on_tel(self, msg) -> None:
        name, d = msg
        if name in ("kinematic", "attitude", "optrange", "flight_mode"):
            self.last[name] = d

    def _open(self) -> None:
        os.makedirs(self.dir, exist_ok=True)
        self.path = os.path.join(self.dir, time.strftime("%Y%m%d_%H%M%S") + "_hold.csv")
        self.f = open(self.path, "w", newline=""); self.w = csv.writer(self.f); self.w.writerow(COLS); self.t0 = self.clock(); self.rows = 0

    def close(self) -> None:
        if self.f is not None:
            try: self.f.close()
            except OSError: pass
        self.f = self.w = None

    def sample(self, sent) -> None:
        """Called once per core tick with the Controls that were just sent."""
        c = self.core
        if not getattr(c, "connected", False) or (not self.explicit and getattr(c, "state_source", "") != "live"):
            self.close(); return
        now = self.clock(); armed = bool(getattr(c.autonomy, "fc_armed", False))
        if now - self._last < (0.1 if (armed or getattr(sent, "arm", False)) else 1.0): return
        self._last = now
        if self.f is None: self._open()
        L = self.last; kin = L.get("kinematic") or {}; att = L.get("attitude") or {}; rg = L.get("optrange") or {}; fm = L.get("flight_mode") or {}
        aid = getattr(c, "pos_aid", None); probe = getattr(c, "probe", None); rc = list(probe.rc) if (probe is not None and probe.rc) else None
        rc_age = (now - probe.t_rc) if (probe is not None and probe.t_rc) else None
        at = lambda i: rc[i] if rc is not None and len(rc) > i else None
        vo = getattr(c, "vo_status", None) or {}
        try: phase = c.director.state().phase
        except Exception: phase = ""                                                       # noqa: BLE001
        deg = lambda r: None if r is None else round(math.degrees(r), 2)
        f = lambda v, n=3: None if v is None else round(v, n)
        self.w.writerow([round(now - self.t0, 2), getattr(c, "state_source", ""), phase, int(armed), fm.get("mode", ""), int(bool(getattr(c.autonomy, "explore", False))),
                         int(bool(getattr(sent, "arm", False))), _us(getattr(sent, "aux5", None)), _us(getattr(sent, "aux7", None)), _us(getattr(sent, "aux9", None)),
                         _us(getattr(sent, "aux10", None)), _us(getattr(sent, "aux11", None)),
                         f(aid[0]) if aid else None, f(aid[1]) if aid else None, f(now - aid[2], 2) if aid else None,
                         at(4), at(8), at(10), at(12), at(13), at(14), f(rc_age, 1),
                         f(kin.get("vx_mps")), f(kin.get("vy_mps")), f(rg.get("distance_m")), deg(att.get("pitch_rad")), deg(att.get("roll_rad")),
                         int(bool(vo.get("anchor"))), int(bool(vo.get("fresh"))), vo.get("text", "")])
        self.rows += 1
        if self.rows % 20 == 0: self.f.flush()
