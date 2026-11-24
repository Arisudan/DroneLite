"""v4.7.2 VO aid on RC channels: CH13 forward, CH14 right (1500 us = 0, 1 us = 1 mm, +-0.40 m), CH15 > 1900 us = valid; only while fresh and armed."""
import time
import unittest

from nimbus_pc.crsf import CRSF_MID, CRSF_MIN
from nimbus_pc.mapper import AID_FLAG_TICKS, aid_to_ticks
from nimbus_pc.sender import Controls


def us(t):
    return 5.0 * t / 8.0 + 880.0


class TestRcAid(unittest.TestCase):
    def test_encoding_and_defaults(self):
        self.assertAlmostEqual(us(aid_to_ticks(0.0)), 1500.0, delta=0.7)
        self.assertAlmostEqual(us(aid_to_ticks(0.123)), 1623.0, delta=0.7)
        self.assertAlmostEqual(us(aid_to_ticks(-0.2)), 1300.0, delta=0.7)
        self.assertAlmostEqual(us(aid_to_ticks(5.0)), 1900.0, delta=0.7)              # clamped to +-0.40 m
        self.assertGreater(us(AID_FLAG_TICKS), 1900.0)
        ch = Controls().to_channels(False)
        self.assertEqual((ch[12], ch[13], ch[14]), (CRSF_MID, CRSF_MID, CRSF_MID))     # no aid: all three at the centre (the FC needs > 1900 us on CH15), the idle frame stays the Nimbus one
        ch = Controls(aux9=aid_to_ticks(0.1), aux10=aid_to_ticks(-0.05), aux11=AID_FLAG_TICKS).to_channels(False)
        self.assertAlmostEqual(us(ch[12]), 1600, delta=0.7); self.assertAlmostEqual(us(ch[13]), 1450, delta=0.7); self.assertGreater(us(ch[14]), 1900)

    def test_core_puts_a_fresh_aid_on_the_channels_only_while_armed(self):
        from nimbus_core.core import AID_FRESH_S, NimbusCore
        c = NimbusCore(); c.connect_sim()
        try:
            sent = []
            orig = c.sess.set_controls
            c.sess.set_controls = lambda x: (sent.append(x), orig(x))[1]
            c.external_controls = Controls(arm=False); c.source = "external"
            c.set_pos_aid(0.05, -0.02); c.tick()
            self.assertEqual(sent[-1].aux11, CRSF_MID)                                  # not armed: no aid
            c.external_controls = Controls(arm=True); c.set_pos_aid(0.05, -0.02); c.tick()
            last = c.last_controls; self.assertTrue(last.arm)
            self.assertEqual(last.aux11, AID_FLAG_TICKS); self.assertAlmostEqual(us(last.aux9), 1550, delta=0.7); self.assertAlmostEqual(us(last.aux10), 1480, delta=0.7)
            c.pos_aid = (0.05, -0.02, time.monotonic() - AID_FRESH_S - 0.1); c.tick()
            self.assertEqual(c.last_controls.aux11, CRSF_MID)                           # stale: no aid
        finally:
            c.disconnect() if hasattr(c, "disconnect") else None


if __name__ == "__main__":
    unittest.main()
