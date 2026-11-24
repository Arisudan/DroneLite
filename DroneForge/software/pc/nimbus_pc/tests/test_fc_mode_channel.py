"""Flight mode on AUX7 (ch11) for firmware v4.2: encoding, what the FC-hold controller sends, and the refusal of STABILIZE."""
import unittest

from nimbus_pc.mapper import MODE_ALT_TICKS, MODE_POS_TICKS, mode_to_aux7
from nimbus_pc.sender import CRSF_MID, CRSF_MIN, Controls
from nimbus_pc.hover.fc_hold_controller import FcHoldController


def us(ticks):
    return 5.0 * ticks / 8.0 + 880.0


class ModeChannel(unittest.TestCase):
    def test_ticks_sit_inside_the_firmware_windows(self):
        alt = lambda t: 1100 <= us(t) < 1400                            # firmware: 1100..1399 us = ALT HOLD, everything else = POS HOLD
        self.assertTrue(alt(MODE_ALT_TICKS)); self.assertFalse(alt(MODE_POS_TICKS))
        self.assertFalse(alt(CRSF_MIN)); self.assertFalse(alt(CRSF_MID))   # minimum and the idle value of an unset channel = POS HOLD (an older PC keeps today's behaviour)
        self.assertEqual(mode_to_aux7("alt_hold"), MODE_ALT_TICKS); self.assertEqual(mode_to_aux7("pos_hold"), MODE_POS_TICKS)

    def test_channel_11_carries_aux7_and_default_is_idle(self):
        ch = Controls(aux7=MODE_ALT_TICKS).to_channels(True)
        self.assertEqual(len(ch), 16); self.assertEqual(ch[10], MODE_ALT_TICKS)
        self.assertEqual(Controls().to_channels(True)[10], CRSF_MID)         # idle frame stays identical to Nimbus's
        self.assertEqual(Controls(aux5=1000, aux6=900, aux7=MODE_POS_TICKS).to_channels(True)[8:11], [1000, 900, MODE_POS_TICKS])   # the other channels are untouched

    def test_controller_accepts_alt_and_pos_only(self):
        c = FcHoldController()
        self.assertEqual(c.flight_mode, "pos_hold")
        self.assertFalse(c.set_flight_mode("stabilize")); self.assertEqual(c.flight_mode, "pos_hold")
        self.assertTrue(c.set_flight_mode("alt_hold")); self.assertEqual(c.flight_mode, "alt_hold")
        self.assertFalse(c.set_flight_mode("bogus"))


if __name__ == "__main__":
    unittest.main()
