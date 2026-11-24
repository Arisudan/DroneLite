"""Does the connected FC firmware take the VO position aid (MSP2_DF_POS_AID, 0x3F00, v4.7 ST only)? Asked of the FC itself, not guessed from a version string
(rev1 and v4.7 both report Betaflight 4.5.4).

Betaflight answers every MSP write over CRSF (telemetry/msp_shared.c processMspPacket): an EMPTY reply when it handled the command, ONE byte
(TELEMETRY_MSP_ERROR = 2) when it does not know it. The probe is the aid itself with valid = 0, which v4.7 stores as "no aid" (height_hold.c heightHoldSetPosAid):
the FC keeps holding on its flow, nothing moves. Asked once per radio session (a reconnect asks again). Qt-free."""
from __future__ import annotations

import time

from nimbus_pc.commands import MSP2_DF_POS_AID, encode_msp_pos_aid

PROBE_WAIT_S = 2.5
PROBE_TRIES = 6           # 2026-11-23 real link: only ~1 in 10 MSP writes got its reply back over ELRS (0.7 s when it did): ask several times, accept a late answer

TEXT = {"unknown": "position aid: checking the FC firmware",
        "probing": "position aid: checking the FC firmware",
        "yes": "",
        "no": "position aid OFF: this FC firmware has no VO aid (v4.6-rev1 or older): v4.7 ST is needed",
        "silent": "position aid OFF: no answer from the FC to the aid check over the radio (6 tries)",
        "nolink": "position aid: no FC connection"}


class AidSupport:
    def __init__(self, clock=time.monotonic) -> None:
        self.clock = clock; self._sess = None; self.state = "nolink"; self.tries = 0; self.t_probe = 0.0

    def update(self, sess) -> str:
        """Call regularly (GUI timer) while VO is on. Returns the state: yes / no / silent / probing / nolink."""
        if sess is None or not hasattr(sess, "send_frame") or not hasattr(sess, "take_msp_reply"):
            self._sess = None; self.state = "nolink"; return self.state
        if sess is not self._sess:                                        # a new radio session (reconnect, other FC): ask again
            self._sess = sess; self.state = "unknown"; self.tries = 0
        if self.state in ("yes", "no", "silent"): return self.state
        now = self.clock()
        if self.state == "probing":
            r = sess.take_msp_reply(MSP2_DF_POS_AID)
            if r is not None:
                self.state = "yes" if len(r.get("payload", b"")) == 0 else "no"; return self.state
            if now - self.t_probe < PROBE_WAIT_S: return self.state
            if self.tries >= PROBE_TRIES: self.state = "silent"; return self.state
        if self.tries == 0: sess.take_msp_reply(MSP2_DF_POS_AID)          # drop a reply from before this session's question (later: any answer counts)
        for fr in encode_msp_pos_aid(0.0, 0.0, False): sess.send_frame(fr)
        self.tries += 1; self.t_probe = now; self.state = "probing"
        return self.state

    @property
    def text(self) -> str:
        return TEXT.get(self.state, "")

    @property
    def blocked(self) -> bool:
        """Show the state in amber: the aid cannot be used with this FC."""
        return self.state in ("no", "silent")
