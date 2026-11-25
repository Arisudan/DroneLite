"""The real-drone explorer (realmap/real_explore.py): roam.py on the real topics, no simulator inputs. The logic class is tested without ROS."""
import json
import math
import socket
import time
import unittest

import numpy as np

from nimbus_core.steering import MAX_TILT_STICK, MAX_YAW_STICK, SteerLink
from realmap import real_explore as RE


class _Clock:
    def __init__(self): self.t = 1000.0
    def __call__(self): return self.t


def _args():
    return RE.build_parser().parse_args([])


def _obs(now, near=3.0):
    return {"rows": 6, "cols": 8, "min_m": [near] * 48, "nearest_m": None, "obstacle": False, "floorplan": None, "stamp": now, "latency_ms": 0}


def _core():
    clk = _Clock(); sent = []; core = RE.RealCore(_args(), sent.append, clock=clk)
    return core, clk, sent


def _feed_all(core, clk, x=0.0, y=0.0, yaw=0.0):
    t = clk()
    core.on_fc(x, y, yaw, 0.0, 0.0, t); core.on_att(0.01, -0.02, t); core.on_obs(_obs(t), t)
    core.on_map(20, 20, 0.05, -0.5, -0.5, [-1] * 400, t)


class TestRealExplorerLogic(unittest.TestCase):
    def test_parked_with_centred_sticks_until_the_gcs_says_explore(self):
        core, clk, sent = _core(); _feed_all(core, clk)
        for _ in range(5): clk.t += 0.05; core.step()
        self.assertEqual(core.stage, "wait"); self.assertTrue(all(d == {"roll": 0.0, "pitch": 0.0, "yaw": 0.0} for d in sent) and sent)      # heartbeat only

    def test_it_refuses_to_start_without_fresh_inputs_and_says_why(self):
        core, clk, sent = _core(); core.on_gcs({"explore": True}, clk())
        clk.t += 0.05; core.step(); self.assertEqual(core.stage, "wait"); self.assertTrue(any("no fresh pose" in l for l in core.log))        # nothing arrived yet
        _feed_all(core, clk); core.map_t = None; core.on_gcs({"explore": True}, clk()); clk.t += 3.0; core.on_fc(0, 0, 0, 0, 0, clk()); core.on_obs(_obs(clk()), clk()); core.on_gcs({"explore": True}, clk())
        core.step(); self.assertEqual(core.stage, "wait"); self.assertTrue(any("no map yet" in l for l in core.log))

    def test_it_starts_when_everything_is_fresh_and_steers_within_the_caps(self):
        core, clk, sent = _core(); _feed_all(core, clk); core.on_gcs({"explore": True}, clk()); clk.t += 0.05; core.step()
        self.assertEqual(core.stage, "roam"); outs = []
        for _ in range(40):
            clk.t += 0.05; core.on_fc(0.0, 0.0, 0.0, 0.0, 0.0, clk()); core.on_att(0.0, 0.0, clk()); core.on_obs(_obs(clk()), clk()); core.on_gcs({"explore": True}, clk())
            n = len(sent); core.step(); outs.append(sent[n:])
        d = [x for step in outs for x in step if "yaw" in x]
        self.assertTrue(d); self.assertTrue(any(abs(x["yaw"]) > 0.0 for x in d))                          # the first thing it does is the 360 degree look
        for x in d:                                                                                         # sane numbers; the GCS applies its own caps on top
            self.assertTrue(all(math.isfinite(v) for v in (x["roll"], x["pitch"], x["yaw"])))
            self.assertLessEqual(abs(x["roll"]), 1.0); self.assertLessEqual(abs(x["pitch"]), 1.0)

    def test_stale_input_while_roaming_centres_the_sticks_then_parks_and_tells_the_gcs(self):
        core, clk, sent = _core(); _feed_all(core, clk); core.on_gcs({"explore": True}, clk()); clk.t += 0.05; core.step(); self.assertEqual(core.stage, "roam")
        clk.t += 0.6; core.on_gcs({"explore": True}, clk()); core.on_obs(_obs(clk()), clk()); n = len(sent); core.step()                    # the pose is now 0.6 s old
        self.assertEqual(sent[n], {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}); self.assertEqual(core.stage, "roam")
        for _ in range(70):
            clk.t += 0.05; core.on_obs(_obs(clk()), clk()); core.on_gcs({"explore": True}, clk()); core.step()
        self.assertEqual(core.stage, "parked"); self.assertTrue(any(d.get("done") and "inputs lost" in d.get("reason", "") for d in sent))

    def test_explore_off_pauses_and_on_again_starts_a_fresh_search(self):
        core, clk, sent = _core(); _feed_all(core, clk); core.on_gcs({"explore": True}, clk()); clk.t += 0.05; core.step(); self.assertEqual(core.stage, "roam")
        clk.t += 0.05; core.on_gcs({"explore": False}, clk()); core.step(); self.assertEqual(core.stage, "wait")
        _feed_all(core, clk); core.on_gcs({"explore": True}, clk()); clk.t += 0.05; core.step(); self.assertEqual(core.stage, "roam")

    def test_silent_gcs_means_not_listening(self):
        core, clk, sent = _core(); _feed_all(core, clk); core.on_gcs({"explore": True}, clk()); clk.t += 1.5; _feed_all(core, clk); core.step()      # no answer for 1.5 s
        self.assertEqual(core.stage, "wait"); self.assertFalse(core.gcs_on)

    def test_tilt_comes_from_the_attitude_and_goes_stale(self):
        core, clk, _ = _core(); self.assertEqual(core.tilt(clk()), 0.0); core.on_att(0.05, -0.12, clk()); self.assertAlmostEqual(core.tilt(clk()), 0.12)
        clk.t += 1.5; self.assertEqual(core.tilt(clk()), 0.0)

    def test_the_defaults_match_the_steering_caps(self):
        a = _args(); self.assertLessEqual(a.max_tilt, 11.0); self.assertLessEqual(a.cruise, 0.3)           # the GCS caps the stick at 0.40 = 10.9 deg; real first runs are slow


class TestOverUdpWithTheRealSteerLink(unittest.TestCase):
    def test_gcs_answer_starts_it_and_the_sticks_arrive_in_the_gcs(self):
        link = SteerLink(port=0); self.assertTrue(link.start()); link.active = True
        try:
            port = link._sock.getsockname()[1]; sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); sock.setblocking(False); addr = ("127.0.0.1", port)
            clk = _Clock(); core = None
            def to_gcs(d):
                sock.sendto(json.dumps(d).encode(), addr); time.sleep(0.03)
                while True:
                    try: core.on_gcs(json.loads(sock.recv(512).decode()), clk())
                    except (BlockingIOError, OSError, ValueError): break
            core = RE.RealCore(_args(), to_gcs, clock=clk); _feed_all(core, clk)
            for _ in range(8): clk.t += 0.05; core.step()                                                  # heartbeat -> the GCS answers {"explore": true}
            self.assertEqual(core.stage, "roam")
            for _ in range(10):
                clk.t += 0.05; core.on_fc(0.0, 0.0, 0.0, 0.0, 0.0, clk()); core.on_obs(_obs(clk()), clk()); core.step()
            got = link.latest(time.monotonic()); self.assertIsNotNone(got)
            self.assertLessEqual(abs(got[0]), MAX_TILT_STICK + 1e-9); self.assertLessEqual(abs(got[2]), MAX_YAW_STICK + 1e-9)
        finally:
            link.stop()


if __name__ == "__main__":
    unittest.main()
