import time
import unittest

from nimbus_core import motor_test as MT
from nimbus_core import topics as T
from nimbus_core.core import NimbusCore
from nimbus_pc import commands
from nimbus_pc.crsf import crc8


class FakeSess:
    """Records every frame; answers MSP_MOTOR from `motors`. Never touches hardware."""
    def __init__(self):
        self.frames = []; self.motors = [1000] * 4; self.follow = True; self.sender = type("S", (), {"allow_arm": False})()
    def send_frame(self, f): self.frames.append(bytes(f))
    def request_msp(self, cmd, timeout=0.3):
        import struct; time.sleep(0.01)
        return {"cmd": cmd, "payload": struct.pack("<4H", *self.motors)}
    def set_controls(self, c): pass
    def stop(self): pass


def decode_sent(frames):
    """Reassemble the sent chunked frames into the 8 motor values."""
    body = b"".join(f[6:-1] for f in frames)
    return [int.from_bytes(body[5 + 2 * i:7 + 2 * i], "little") for i in range(8)]


def groups(sess):
    out, cur = [], []
    for f in sess.frames:
        cur.append(f)
        if len(cur) == 4: out.append(decode_sent(cur)); cur = []
    return out


def make(mode="STAB*", lq=100):
    c = NimbusCore(); s = FakeSess(); c.sess = s; c.state_source = "live"
    c.bus.publish(T.TELEMETRY, ("flight_mode", {"mode": mode}), cache=False)
    c.bus.publish(T.TELEMETRY, ("link", {"up_lq": lq}), cache=False)
    return c, s


class TestEncode(unittest.TestCase):
    def test_frames_match_hand_computed_bytes(self):
        fr = commands.encode_msp_set_motor([1000, 1075, 1000, 1000])
        body = bytes.fromhex("00d6001000e8033304e803e803e803e803e803e8031a")
        self.assertEqual(len(fr), 4)
        for i, f in enumerate(fr):
            chunk = body[7 * i:7 * i + 7]; st = 0x40 | (0x10 if i == 0 else 0) | i
            inner = bytes([0x7C, 0xC8, 0xEA, st]) + chunk
            self.assertEqual(f, bytes([0xC8, len(inner) + 1]) + inner + bytes([crc8(inner)]))
        self.assertEqual(fr[0].hex()[:26], "c80c7cc8ea5000d6001000e803")
        self.assertEqual(decode_sent(fr), [1000, 1075, 1000, 1000] + [1000] * 4)
        with self.assertRaises(ValueError): commands.encode_msp_set_motor([999] * 4)
        with self.assertRaises(ValueError): commands.encode_msp_set_motor([2001] * 4)


class TestGates(unittest.TestCase):
    def setUp(self):
        MT.ZERO_GAP_S = 0.02; MT.POLL_S = 0.02

    def test_each_gate_refuses_with_its_reason(self):
        c = NimbusCore(); self.assertIn("Connect", c.motor_test.start(1))
        c, s = make(); c.motor_test.t_status -= 5; self.assertIn("fresh", c.motor_test.refusal())
        c, s = make(mode="STAB"); self.assertEqual(c.motor_test.start(2), "FC is armed: disarm first.")
        c, s = make(lq=40); self.assertIn("40", c.motor_test.start(2))
        c, s = make(); c.ramp.phase = "ramping"; self.assertIn("already running", c.motor_test.start(1))
        c, s = make(); c.state_source = "sim"; self.assertIn("live", c.motor_test.refusal())
        for c_ in (c,): self.assertEqual(s.frames, [])
        c, s = make(); self.assertEqual(c.motor_test.refusal(), ""); self.assertEqual(c.motor_test.start(7), "Pick motor 1 to 4.")

    def test_only_chosen_motor_and_never_above_1100(self):
        c, s = make(); mt = c.motor_test; mt.value_us = 5000
        self.assertEqual(mt.start(3), "")
        g = groups(s)[-1]; self.assertEqual(g[:4], [1000, 1000, 1100, 1000])
        self.assertTrue(all(max(x) <= 1100 for x in groups(s)))
        mt.stop(); self.assertTrue(mt.wait_idle(5))
        self.assertEqual(MT.MOTOR_TEST_US, 1075)

    def test_release_stop_sends_three_zeros_then_watchdog_until_idle(self):
        c, s = make(); mt = c.motor_test; mt.start(2); n0 = len(groups(s))
        s.motors = [1000, 1075, 1000, 1000]                       # readback still spinning
        mt.stop("release"); self.assertEqual(mt.state, "stopping")
        time.sleep(0.4); self.assertEqual(mt.state, "stopping")  # not idle while the readback shows the motor turning
        z = groups(s)[n0:]; self.assertGreaterEqual(len(z), 4); self.assertTrue(all(x == [1000] * 8 for x in z))
        s.motors = [1000] * 4; self.assertTrue(mt.wait_idle(5)); self.assertEqual(mt.state, "idle")

    def test_watchdog_gives_up_after_3s(self):
        MT.WATCHDOG_S = 0.3
        try:
            c, s = make(); mt = c.motor_test; mt.start(1); s.motors = [1075, 1000, 1000, 1000]; mt.stop()
            self.assertTrue(mt.wait_idle(5)); self.assertIn("did not confirm", mt.status_text)
        finally:
            MT.WATCHDOG_S = 3.0

    def test_cap_2_5_s(self):
        c, s = make(); mt = c.motor_test; mt.start(4); mt.t_start -= 2.6; mt.refresh()
        self.assertEqual(mt.state, "stopping"); self.assertIn("press again", mt.status_text); mt.wait_idle(5)

    def test_gate_loss_while_spinning_stops(self):
        c, s = make(); mt = c.motor_test; mt.start(4)
        c.bus.publish(T.TELEMETRY, ("flight_mode", {"mode": "STAB"}), cache=False); mt.refresh()
        self.assertEqual(mt.state, "stopping"); self.assertIn("armed", mt.status_text); mt.wait_idle(5)

    def test_not_confirmed_when_readback_stays_idle(self):
        c, s = make(); mt = c.motor_test; mt.start(2); mt.t_start -= 1.2; mt.refresh()
        self.assertTrue(mt.unconfirmed); self.assertIn("FC did not spin M2", mt.status_text); self.assertIn("not confirmed", mt.status_text)
        self.assertEqual(mt.state, "spinning"); mt.stop(); mt.wait_idle(5)

    def test_confirmed_with_readback(self):
        c, s = make(); mt = c.motor_test; mt.start(2); s.motors = [1000, 1075, 1000, 1000]; time.sleep(0.15)
        mt.t_start -= 1.2; mt.refresh(); self.assertFalse(mt.unconfirmed); mt.stop(); s.motors = [1000] * 4; mt.wait_idle(5)

    def test_safe_and_disconnect_stop(self):
        c, s = make(); c.motor_test.start(1); c.safe_now(); self.assertEqual(c.motor_test.state, "stopping")
        s.motors = [1000] * 4; c.motor_test.wait_idle(5)
        c, s = make(); c.motor_test.start(1); n = len(s.frames); c.disconnect()
        self.assertEqual(c.motor_test.state, "idle"); self.assertGreaterEqual((len(s.frames) - n) // 4, 3)
        self.assertTrue(all(x == [1000] * 8 for x in groups(s)[1:]))


if __name__ == "__main__":
    unittest.main()
