"""EXPLORE steering input (nimbus_core/steering.py) and its gate in the autonomy: sticks only while EXPLORE is on and the FC holds; stale = centred; capped; LAND ends it."""
import dataclasses
import json
import socket
import time
import unittest

from nimbus_core import fc_hold_model as M
from nimbus_core import settings as S
from nimbus_core.core import NimbusCore
from nimbus_core.steering import MAX_TILT_STICK, MAX_YAW_STICK, STALE_S, SteerLink


class TestSteerLink(unittest.TestCase):
    def test_fresh_stale_caps_and_done(self):
        s = SteerLink(port=0)
        self.assertIsNone(s.latest(0.0))
        self.assertTrue(s.feed(json.dumps({"roll": 0.1, "pitch": 2.0, "yaw": -3.0}).encode(), now=10.0))
        self.assertEqual(s.latest(10.1), (0.1, MAX_TILT_STICK, -MAX_YAW_STICK))
        self.assertIsNone(s.latest(10.0 + STALE_S + 0.01))                       # stale -> centred sticks (None)
        self.assertTrue(s.feed(b'{"roll": NaN, "pitch": "x"}', now=20.0))
        self.assertEqual(s.latest(20.0), (0.0, 0.0, 0.0))                         # garbage values -> 0, never NaN
        self.assertFalse(s.feed(b"not json", now=21.0)); self.assertFalse(s.feed(b"[1, 2]", now=21.0))
        s.feed(b'{"done": true}', now=22.0); self.assertTrue(s.done); self.assertIsNone(s.latest(22.0))

    def test_udp_round_trip_and_answer(self):
        s = SteerLink(port=0)
        self.assertTrue(s.start())
        try:
            port = s._sock.getsockname()[1]; s.active = True
            c = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); c.settimeout(2.0)
            c.sendto(json.dumps({"roll": 0.0, "pitch": 0.2, "yaw": 0.0}).encode(), ("127.0.0.1", port))
            self.assertEqual(json.loads(c.recv(512).decode()), {"explore": True})
            self.assertEqual(s.latest(), (0.0, 0.2, 0.0))
            c.close()
        finally:
            s.stop()


@unittest.skipUnless(M.lib_path(), "libhh.so not built (tools_scripts/fc_hold_sim/build.sh)")
class TestExploreGate(unittest.TestCase):
    def _wait(self, core, d, cond, timeout, what):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            core.tick(); time.sleep(0.01)
            if cond(): return
        self.fail(f"timeout waiting for {what}: phase {d.state().phase}, {d.state().message}")

    def test_explore_only_while_holding_and_land_ends_it(self):
        core = NimbusCore(); d = core.director; au = d._au
        st = dataclasses.replace(au.settings, hover_height=0.5, ceiling_m=0.8, thr_hover0=0.66, thr_max=0.85, measured=True, batt_ref_v=13.8, profile="fc")
        self.assertTrue(d.apply_settings(st, persist=False).ok)
        core.connect_sim(params=dataclasses.replace(S.plant_params(st), hover_thr=0.72))
        au.steer = SteerLink(port=0)
        try:
            self._wait(core, d, lambda: d.preflight_input().get("range_m") is not None and (d.preflight_input().get("link_lq") or 0) > 0, 20, "telemetry")
            for _ in range(400): core.tick(); time.sleep(0.01)
            self.assertFalse(d.explore(True).ok)                                   # on the ground: refused
            self.assertTrue(d.arm().ok); self._wait(core, d, lambda: d.state().phase == "armed", 25, "armed")
            self.assertTrue(d.takeoff(0.5).ok); self._wait(core, d, lambda: d.state().phase == "holding", 80, "holding")
            self.assertTrue(d.explore(True).ok); self.assertTrue(au.explore)
            au.steer.feed(json.dumps({"roll": 0.0, "pitch": 0.25, "yaw": 0.1}).encode())
            c = au.controls(time.monotonic())
            self.assertAlmostEqual(c.pitch, 0.25); self.assertAlmostEqual(c.yaw, 0.1); self.assertTrue(c.arm)
            self.assertGreater(c.aux5, 172)                                        # the FC height hold request is kept
            self.assertEqual(c.aux7, 512)                                          # ALT HOLD while steering: the FC sideways loop is paused
            time.sleep(STALE_S + 0.05)
            c = au.controls(time.monotonic()); self.assertEqual((c.roll, c.pitch, c.yaw), (0.0, 0.0, 0.0))   # stale: centred
            self.assertEqual(c.aux7, 1632)                                         # ... and POS HOLD: the FC holds the position
            self.assertTrue(d.land().ok); self.assertFalse(au.explore)
            self.assertFalse(d.explore(True).ok)                                   # landing: refused
            self._wait(core, d, lambda: d.state().phase in ("landed", "ready"), 60, "landed")
        finally:
            au.steer.stop(); core.disconnect()


if __name__ == "__main__":
    unittest.main()
