"""RC channel test: do all 16 channels the GCS sends really arrive at the FC, intact? (2026-11-25)

Why: with the ELRS switch mode Wide / Hybrid the FC silently reads CH13-16 as 1500 us and CH6-12 in 15.9 us steps (measured 2026-11-24): the VO aid (CH13-15) never arrived and the
height target (CH9) came ~22 mm high; in the in-between state 100Hz Full + 8ch CH9 read 1500 us = "hold ON, 1.09 m". A channel the FC reads as its idle value looks exactly like a channel
that was sent as idle, so the only proof is to send a distinctive value and read it back (MSP_RC over the radio, nimbus_core/radio_fc.py).

The test sends a PATTERN on CH9-CH16 while the FC is DISARMED and nothing flies. None of these channels does anything on a disarmed FC (the hold, the mode switch and the aid are only
read while armed and holding; CH8 = beeper is left alone, CH5-7 are the arm / ANGLE / turtle switches and stay as they are). Armed = the test is cancelled at once. Qt-free."""
from __future__ import annotations

import time
from typing import Optional

# (name, what the FC does with it, what rate it needs)
ROLES = {
    1: ("ROLL", "roll stick (the explorer's steering)", "every frame, 100 Hz"),
    2: ("PITCH", "pitch stick (the explorer's steering)", "every frame, 100 Hz"),
    3: ("THROTTLE", "throttle (minimum while the FC holds the height)", "every frame, 100 Hz"),
    4: ("YAW", "yaw stick", "every frame, 100 Hz"),
    5: ("ARM", "arm switch AUX1 (1700 us or more = armed)", "every frame; older than 0.25 s = the GCS disarms"),
    6: ("ANGLE", "ANGLE / HORIZON AUX2 (low = ANGLE)", "held, changes rarely"),
    7: ("TURTLE", "flip-over (turtle) switch AUX3", "held, changes rarely"),
    8: ("BEEPER", "beeper AUX4 (above 1700 us beeps)", "held, changes rarely"),
    9: ("HEIGHT", "FC height hold + target AUX5 (1150-1950 us = 0-2.5 m, 1960 or more = LAND)", "on a change; held otherwise (2 mm steps need 10-bit)"),
    10: ("HOVER REF", "hover-reference throttle for the FC take-off AUX6", "held while the hold is on"),
    11: ("MODE", "flight mode AUX7 (1100-1400 us = ALT HOLD, else POS HOLD)", "10 Hz is plenty"),
    12: ("SPARE", "no FC function (used by the channel test)", "-"),
    13: ("AID FWD", "VO position aid, forward AUX9 (1500 us = 0, 1 us = 1 mm)", "10 Hz, fresh under 0.25 s; needs 10-bit (100Hz Full)"),
    14: ("AID RIGHT", "VO position aid, right AUX10", "10 Hz, fresh under 0.25 s; needs 10-bit"),
    15: ("AID VALID", "VO aid valid flag AUX11 (above 1900 us = valid)", "10 Hz with the aid"),
    16: ("SPARE", "no FC function (used by the channel test)", "-"),
}
TEST_FIELDS = {9: "aux5", 10: "aux6", 11: "aux7", 12: "aux8", 13: "aux9", 14: "aux10", 15: "aux11", 16: "aux12"}      # channel -> Controls field
PATTERN = {9: 1337, 10: 1213, 11: 1411, 12: 1432, 13: 1537, 14: 1463, 15: 1111, 16: 1666}                          # us; 15 stays below 1900: even armed it would not be a valid aid
EXACT_US = 3            # full resolution (10-bit) arrives within this
COARSE_US = 17          # Wide / Hybrid switch modes: 15.9 us steps


def us_to_ticks(us: float) -> int:
    return int(round((us - 880.0) * 8.0 / 5.0))


def ticks_to_us(t: float) -> float:
    return 5.0 * t / 8.0 + 880.0


def judge(fc: list) -> tuple:
    """(ok, text, rows) for one read-back of the FC's rcData against PATTERN. rows = [(channel, sent_us, fc_us, error_us, verdict)]."""
    rows = []
    for ch, want in PATTERN.items():
        got = fc[ch - 1] if len(fc) >= ch else None
        if got is None: rows.append((ch, want, None, None, "missing")); continue
        err = got - want
        rows.append((ch, want, got, err, "ok" if abs(err) <= EXACT_US else ("coarse" if abs(err) <= COARSE_US else "WRONG")))
    bad = [r for r in rows if r[4] in ("WRONG", "missing")]; coarse = [r for r in rows if r[4] == "coarse"]
    if not bad and not coarse:
        return True, f"CH9-16 reach the FC intact (largest error {max(abs(r[3]) for r in rows)} us)", rows
    aid_bad = [r for r in bad if r[0] in (13, 14, 15)]
    if len(aid_bad) == 3 and len(bad) >= 3 and all(r[2] is not None and abs(r[2] - 1500) <= 2 for r in aid_bad):
        return False, "CH13-16 are NOT carried (the FC reads them as 1500 us): the VO aid cannot reach it. Set ELRS 100Hz Full + 16ch", rows
    if coarse and not bad:
        return False, "CH9-16 arrive only in coarse steps (ELRS Wide / Hybrid switch mode): the height target and the aid are off. Set ELRS 100Hz Full + 16ch", rows
    return False, "channel(s) differ at the FC: " + ", ".join(f"CH{r[0]} sent {r[1]} got {r[2]}" for r in bad[:4]), rows


class RcChannelCheck:
    TIMEOUT_S = 15.0
    SETTLE_S = 0.25         # the pattern needs a few frames to reach the FC before a read-back can count
    NEED_OK = 2             # consecutive good read-backs
    NEED_BAD = 3            # consecutive bad read-backs = a real failure (do not wait for the timeout)

    def __init__(self, clock=time.monotonic) -> None:
        self.clock = clock
        self.auto_done = False
        self.reset()

    def reset(self, note: str = "not tested yet") -> None:
        self.state = "idle"; self.detail = note; self.t0: Optional[float] = None; self.t_done: Optional[float] = None
        self.rows: list = []; self._ok = self._bad = 0; self._last_req = None; self.runs = 0

    def start(self) -> None:
        self.state = "running"; self.detail = "testing: sending a pattern on CH9-16 and reading it back"; self.t0 = self.clock(); self.t_done = None
        self.rows = []; self._ok = self._bad = 0; self._last_req = None; self.runs += 1

    def cancel(self, why: str) -> None:
        if self.state == "running":
            self.state = "idle"; self.detail = f"cancelled ({why})"; self.t0 = None

    def overrides(self) -> Optional[dict]:
        """Controls fields to replace while the test runs (None otherwise)."""
        return {TEST_FIELDS[ch]: us_to_ticks(us) for ch, us in PATTERN.items()} if self.state == "running" else None

    def feed(self, rc: Optional[list], req_t: Optional[float]) -> None:
        """Newest MSP_RC read-back and the time ITS REQUEST was sent. Counts only if the request was sent after the pattern had time to arrive."""
        if self.state != "running":
            return
        now = self.clock()
        if rc and req_t is not None and req_t != self._last_req and req_t >= self.t0 + self.SETTLE_S:
            self._last_req = req_t
            ok, text, rows = judge(rc); self.rows = rows
            self._ok, self._bad = (self._ok + 1, 0) if ok else (0, self._bad + 1)
            if self._ok >= self.NEED_OK: self._finish("ok", text, now); return
            if self._bad >= self.NEED_BAD: self._finish("fail", text, now); return
        if now - self.t0 > self.TIMEOUT_S:
            self._finish("silent", "no channel read-back from the FC over the radio in %.0f s (MSP replies are slow or lost): not a failure, run it again" % self.TIMEOUT_S, now)

    def _finish(self, state: str, text: str, now: float) -> None:
        self.state, self.detail, self.t_done = state, text, now

    @property
    def blocking(self) -> bool:
        return self.state == "fail"
