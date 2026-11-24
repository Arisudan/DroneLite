"""fc_caps.AidSupport over the real radio session against the emulated FC: v4.7 (handles MSP2_DF_POS_AID: empty reply), older firmware (one error byte), and no answer."""
import time
import unittest

from nimbus_gcs.fc_caps import AidSupport
from nimbus_pc import commands
from nimbus_pc.emulator import DongleEmulator
from nimbus_pc.session import Session


def _run(v47: bool, telemetry: bool = True):
    emu = DongleEmulator()
    if v47: emu.KNOWN_WRITES = set(emu.KNOWN_WRITES) | {commands.MSP2_DF_POS_AID}
    emu.telemetry_enabled = telemetry; emu.start()
    t = [0.0]; a = AidSupport(clock=lambda: t[0])
    try:
        with Session(emu.path, ping=False, listen_only=True) as s:
            for _ in range(200):                                  # 20 s of 0.1 s GUI ticks (fake clock: 0.1 s per tick)
                st = a.update(s)
                if st in ("yes", "no", "silent"): break
                time.sleep(0.1); t[0] += 0.1
    finally:
        emu.stop()
    return a


class TestAidSupport(unittest.TestCase):
    def test_v47_takes_the_aid(self):
        a = _run(True); self.assertEqual(a.state, "yes"); self.assertFalse(a.blocked); self.assertEqual(a.tries, 1)

    def test_older_firmware_is_told_and_blocked(self):
        a = _run(False); self.assertEqual(a.state, "no"); self.assertTrue(a.blocked); self.assertIn("v4.7 ST is needed", a.text)

    def test_no_answer_is_blocked_after_all_tries(self):
        from nimbus_gcs.fc_caps import PROBE_TRIES
        a = _run(True, telemetry=False); self.assertEqual(a.state, "silent"); self.assertTrue(a.blocked); self.assertEqual(a.tries, PROBE_TRIES)

    def test_no_link(self):
        a = AidSupport(); self.assertEqual(a.update(None), "nolink"); self.assertFalse(a.blocked)

    def test_the_probe_carries_valid_0(self):                      # v4.7 stores it as "no aid": the FC keeps holding on its flow
        sent = []

        class S:
            def send_frame(self, f): sent.append(f)
            def take_msp_reply(self, c): return None

        AidSupport().update(S())
        asm = commands.MspResponseAssembler(commands.TYPE_MSP_WRITE)
        from nimbus_pc.crsf import FrameParser
        p = FrameParser(); msg = None
        for f in sent:
            for fr in p.feed(f):
                msg = asm.feed(fr) or msg
        self.assertEqual(msg["cmd"], commands.MSP2_DF_POS_AID); self.assertFalse(commands.decode_msp_pos_aid(msg["payload"])["valid"])


if __name__ == "__main__":
    unittest.main()
