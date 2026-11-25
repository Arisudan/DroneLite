import time
import unittest

from nimbus_core import topics as T
from nimbus_core.bus import TopicBus
from nimbus_core.core import NimbusCore
from nimbus_core.director import FlightDirector, evaluate_preflight, _check_sig
from nimbus_core.link_supervisor import LinkSupervisor
from nimbus_core.notifications import Notifications, infer_level
from nimbus_core.tests.test_director import GOOD


class FakeCore:
    def __init__(self):
        self.bus, self.connected = TopicBus(), True


class TestLinkSupervisor(unittest.TestCase):
    def setUp(self):
        self.t = [100.0]
        self.core = FakeCore()
        self.ls = LinkSupervisor(self.core, clock=lambda: self.t[0])
        self.log = []
        self.core.bus.subscribe(T.LINK_STATE, lambda m: self.log.append(m["state"]))

    def rx(self, lq=100):
        self.core.bus.publish(T.FRAME_IN, object(), cache=False)
        self.core.bus.publish(T.TELEMETRY, ("link", {"up_lq": lq}), cache=False)

    def step(self, dt, lq=100, rx=True):
        self.t[0] += dt
        if rx:
            self.rx(lq)
        return self.ls.update()

    def test_flow(self):
        self.assertEqual(self.ls.update(), "connecting")
        self.assertEqual(self.step(0.1), "connected")
        self.assertEqual(self.step(0.1, lq=40), "degraded")
        self.assertEqual(self.step(0.5), "degraded")             # good data again, but needs 1 s
        for _ in range(11):
            self.step(0.1)
        self.assertEqual(self.ls.state(), "connected")
        self.assertEqual(self.step(0.7, rx=False), "degraded")     # silence > 0.5 s
        self.assertEqual(self.step(1.5, rx=False), "lost")         # > 2 s total
        self.assertEqual(self.step(0.1), "lost")                   # hysteresis: 1 s good needed
        for _ in range(11):
            self.step(0.1)
        self.assertEqual(self.ls.state(), "connected")
        self.assertEqual(self.log, ["connected", "degraded", "connected", "degraded", "lost", "connected"])   # only changes

    def test_lq_zero_lost(self):
        self.step(0.1)
        for _ in range(5):
            self.step(0.1, lq=0)
        self.assertEqual(self.ls.state(), "degraded")
        for _ in range(7):
            self.step(0.1, lq=0)
        self.assertEqual(self.ls.state(), "lost")
        self.assertGreaterEqual(self.ls.since_s, 0.0)

    def test_disconnect_resets(self):
        self.step(0.1)
        self.core.connected = False
        self.assertEqual(self.ls.update(), "offline")


class TestNotifications(unittest.TestCase):
    def test_ring_and_unread(self):
        core = NimbusCore()
        seen = []
        core.bus.subscribe(T.NOTIFY, seen.append)
        core.event("hello")
        core.notify("Battery reads 13.8 V (warning only)", "caution", "battery")
        self.assertEqual([n.level for n in core.notifications.unread()], ["info", "caution"])
        self.assertEqual(len(seen), 2)
        core.notifications.mark_read()
        self.assertEqual(core.notifications.unread(), [])
        for i in range(250):
            core.event(f"e{i}")
        self.assertEqual(len(core.notifications.all()), 200)
        self.assertEqual(len(core.notifications.unread()), 200)

    def test_levels(self):
        self.assertEqual(infer_level("SAFETY [alarm] inverted"), "alarm")
        self.assertEqual(infer_level("pilot LET GO of the stick"), "alarm")
        self.assertEqual(infer_level("FC no longer armed"), "alarm")
        self.assertEqual(infer_level("take-off REFUSED"), "caution")
        self.assertEqual(infer_level("battery low"), "caution")
        self.assertEqual(infer_level("SAFE pressed: disarm"), "info")
        self.assertEqual(infer_level("connected to x"), "info")


class TestPreflightTopic(unittest.TestCase):
    def test_hints_and_goto(self):
        cs = {c.id: c for c in evaluate_preflight(GOOD)}
        self.assertEqual(cs["rest_tilt"].goto, ("SETUP", "Calibration"))
        self.assertIn("Level", cs["rest_tilt"].fix_hint)
        self.assertEqual(cs["range"].goto, ("SETUP", "Sensors"))
        self.assertEqual(cs["link"].goto, ("SETUP", "Radio"))
        self.assertEqual(cs["battery"].goto, ("SETTINGS", "Settings"))
        self.assertIsNone(cs["fc_usb"].goto)
        self.assertEqual(cs["ceiling"].goto, ("SETTINGS", "Settings"))
        self.assertEqual(cs["rc_channels"].goto, ("DIAGNOSTICS", "Diagnostics"))
        self.assertTrue(all(c.fix_hint for c in cs.values()))

    def test_sig_ignores_numbers(self):
        a = _check_sig(evaluate_preflight(dict(GOOD, link_lq=97)))
        self.assertEqual(a, _check_sig(evaluate_preflight(dict(GOOD, link_lq=98))))
        self.assertNotEqual(a, _check_sig(evaluate_preflight(dict(GOOD, link_lq=10))))


class TestLifecycleSim(unittest.TestCase):
    def setUp(self):
        self.core = NimbusCore()
        self.core.connect_sim()
        self.addCleanup(self.core.disconnect)

    def pump(self, cond, timeout=15.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.core.tick()
            if cond():
                return True
            time.sleep(0.02)
        return False

    def test_arm_confirmed_and_topic_once(self):
        d, flips = self.core.director, []
        self.core.bus.subscribe(T.PREFLIGHT, flips.append)
        self.pump(lambda: d.state().link_state == "connected", 5)
        r = d.arm()
        self.assertTrue(r.ok, r.message)
        self.assertEqual((r.status, d.state().command.status), ("issued", "issued"))
        self.assertTrue(self.pump(lambda: d.command_status(r.command_id).status != "issued"))
        self.assertEqual(d.command_status(r.command_id).status, "confirmed")
        self.assertEqual(d.state().mission_phase, "armed")
        n = len(flips)
        for _ in range(100):
            self.core.tick()
        self.assertEqual(len(flips), n)                      # no flip, no publish
        d.safe()
        s = d.state()
        self.assertEqual(s.mission_phase, "safe")
        self.assertEqual(s.command.name, "safe"); self.assertEqual(s.command.status, "confirmed")

    def test_denied_and_summary(self):
        d = self.core.director
        r = d.land()
        self.assertEqual((r.ok, r.status), (False, "denied"))
        self.assertEqual(d.command_status(r.command_id).status, "denied")
        ok, total, cautions = d.preflight_summary()
        self.assertEqual(total, 12); self.assertLessEqual(ok, total); self.assertIsInstance(cautions, int)      # 12 since 2026-11-25 (the RC channel row)

    def test_arm_timeout_no_retry(self):
        d = self.core.director
        r = d.arm()
        self.assertTrue(r.ok)
        t0 = d.clock()
        d.clock = lambda: t0 + 100                           # past the deadline
        self.core.sess.set_controls = lambda c: None
        d._au.fc_mode = ""                                   # FC never reports armed
        d._update_commands(d.clock())
        c = d.command_status(r.command_id)
        if c.status != "confirmed":
            self.assertEqual(c.status, "timed_out")
            self.assertIn("not retried", c.message)
        self.assertEqual(d._cmd_seq, r.command_id)           # nothing re-issued


if __name__ == "__main__":
    unittest.main()
