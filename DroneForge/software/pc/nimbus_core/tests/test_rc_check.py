"""RC channel test: the verdicts, the state machine, the channel order, the pre-flight row and the core hook (no hardware)."""
import struct
import unittest
from types import SimpleNamespace

from nimbus_core import rc_check as R
from nimbus_core.core import NimbusCore
from nimbus_core.director import evaluate_preflight
from nimbus_core.radio_fc import decode_rc
from nimbus_pc.sender import CRSF_MID, Controls


def _fc(**diff):
    rc = [1500] * 16
    for ch, us in R.PATTERN.items(): rc[ch - 1] = us
    for k, v in diff.items(): rc[int(k[2:]) - 1] = v
    return rc


class TestJudge(unittest.TestCase):
    def test_all_arrive(self):
        ok, text, rows = R.judge(_fc(ch13=1538, ch9=1336)); self.assertTrue(ok); self.assertIn("intact", text); self.assertEqual(len(rows), 8)

    def test_wide_mode_cannot_carry_the_aid(self):
        ok, text, _ = R.judge(_fc(ch13=1500, ch14=1500, ch15=1500, ch16=1500, ch9=1344)); self.assertFalse(ok); self.assertIn("NOT carried", text); self.assertIn("100Hz Full", text)

    def test_coarse_steps_are_caught(self):
        ok, text, _ = R.judge(_fc(ch9=1337 + 11, ch12=1432 - 9)); self.assertFalse(ok); self.assertIn("coarse", text)

    def test_a_wrong_channel_is_named(self):
        ok, text, rows = R.judge(_fc(ch10=1500)); self.assertFalse(ok); self.assertIn("CH10", text)
        ok, text, _ = R.judge([1500] * 12); self.assertFalse(ok)                                         # a short read-back: the missing channels fail

    def test_pattern_cannot_do_anything_on_the_fc(self):
        self.assertTrue(R.PATTERN[15] < 1900)                                                          # even armed it would not be a valid aid
        self.assertNotIn(8, R.PATTERN); self.assertNotIn(5, R.PATTERN)                                # CH8 beeper, CH5 arm are never touched
        self.assertEqual(set(R.PATTERN), set(R.TEST_FIELDS)); self.assertEqual(sorted(R.ROLES), list(range(1, 17)))


class TestStateMachine(unittest.TestCase):
    def setUp(self):
        self.t = [100.0]; self.c = R.RcChannelCheck(clock=lambda: self.t[0])

    def test_ok_after_two_good_readbacks_that_were_requested_after_the_pattern(self):
        c = self.c; self.assertIsNone(c.overrides()); c.start(); ov = c.overrides()
        self.assertEqual(set(ov), set(R.TEST_FIELDS.values()))
        for ch, f in R.TEST_FIELDS.items(): self.assertAlmostEqual(R.ticks_to_us(ov[f]), R.PATTERN[ch], delta=1.0)     # the pattern survives the tick conversion
        c.feed(_fc(), 99.9); self.assertEqual(c.state, "running")                                       # requested BEFORE the test: ignored
        c.feed(_fc(), 100.1); self.assertEqual(c.state, "running")                                      # inside the settling time: ignored
        c.feed(_fc(), 100.5); self.assertEqual(c.state, "running")                                      # one good
        c.feed(_fc(), 100.5); self.assertEqual(c.state, "running")                                      # the same reply again does not count twice
        c.feed(_fc(), 100.9); self.assertEqual(c.state, "ok"); self.assertIsNone(c.overrides()); self.assertFalse(c.blocking)

    def test_three_bad_readbacks_fail_and_block(self):
        c = self.c; c.start()
        for i in range(3): c.feed(_fc(ch13=1500, ch14=1500, ch15=1500, ch16=1500), 100.5 + i * 0.4)
        self.assertEqual(c.state, "fail"); self.assertTrue(c.blocking); self.assertIn("NOT carried", c.detail)

    def test_a_good_one_in_between_resets_the_bad_count(self):
        c = self.c; c.start(); bad = _fc(ch10=1500)
        c.feed(bad, 100.5); c.feed(bad, 100.9); c.feed(_fc(), 101.3); c.feed(bad, 101.7); c.feed(bad, 102.1)
        self.assertEqual(c.state, "running")

    def test_silence_is_not_a_failure(self):
        c = self.c; c.start(); self.t[0] += 16.0; c.feed(None, None)
        self.assertEqual(c.state, "silent"); self.assertFalse(c.blocking); self.assertIn("not a failure", c.detail)

    def test_cancel(self):
        c = self.c; c.start(); c.cancel("arming"); self.assertEqual(c.state, "idle"); self.assertIn("cancelled", c.detail); self.assertIsNone(c.overrides())


class TestPlumbing(unittest.TestCase):
    def test_decode_rc_swaps_yaw_and_throttle_to_crsf_order(self):
        v = decode_rc(struct.pack("<8H", 1500, 1500, 1500, 988, 988, 0, 0, 0))      # Betaflight order: roll, pitch, yaw, throttle
        self.assertEqual(v[:4], [1500, 1500, 988, 1500])

    def test_decode_rc_and_the_channel_order(self):
        self.assertEqual(decode_rc(struct.pack("<16H", *range(1000, 1016)))[12], 1012); self.assertIsNone(decode_rc(b"\x01\x02"))
        ch = Controls(aux8=1700, aux12=800, aux5=900, aux9=1000).to_channels(False)
        self.assertEqual((ch[8], ch[11], ch[12], ch[15]), (900, 1700, 1000, 800))                         # CH9 CH12 CH13 CH16

    def test_preflight_row(self):
        base = dict(connected=True, simulated=False, link_lq=100, telemetry_age_s=0.1, fc_armed=False, rest_tilt_deg=0.1, range_m=0.05, range_age_s=0.1, autonomy_idle=True,
                    safety_latch=None, battery_v=14.0, fc_usb=False, height_m=0.5, ceiling_m=2.0)
        row = lambda **k: {c.id: c for c in evaluate_preflight(dict(base, **k))}["rc_channels"]
        self.assertTrue(row(rc_check="ok").ok); self.assertTrue(row().ok)                                # key absent: unknown, not a failure
        for s in ("idle", "running", "silent"): r = row(rc_check=s, rc_detail="x"); self.assertFalse(r.ok); self.assertFalse(r.blocking)
        r = row(rc_check="fail", rc_detail="CH13-16 are NOT carried"); self.assertFalse(r.ok); self.assertTrue(r.blocking); self.assertIn("NOT carried", r.detail)
        self.assertTrue(row(simulated=True, rc_check="fail").ok)

    def test_core_hook_starts_once_applies_the_pattern_and_never_while_armed(self):
        core = NimbusCore(); core.state_source = "live"; core.sess = SimpleNamespace(); core.probe = SimpleNamespace(rc=None, rc_req_t=None, replies=0)
        c = core._rc_channel_test(Controls()); self.assertEqual(core.rc_check.state, "idle")             # the radio MSP path has not answered yet: do not start
        core.probe.replies = 2; c = core._rc_channel_test(Controls())
        self.assertEqual(core.rc_check.state, "running"); self.assertEqual(c.to_channels(False)[12], R.us_to_ticks(R.PATTERN[13]))
        self.assertEqual(c.to_channels(False)[4], 172)                                                    # CH5 arm stays low
        c = core._rc_channel_test(Controls(arm=True))                                                     # arming cancels it at once and sends nothing extra
        self.assertEqual(core.rc_check.state, "idle"); self.assertEqual(c.to_channels(True)[12], CRSF_MID)
        core.rc_check.reset(); c = core._rc_channel_test(Controls()); self.assertEqual(core.rc_check.state, "idle")      # auto-start happens once per radio session
        self.assertTrue(core.start_rc_check() is True and core.rc_check.state == "running")


if __name__ == "__main__":
    unittest.main()
