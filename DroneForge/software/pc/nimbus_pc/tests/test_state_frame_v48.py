"""Firmware v4.8 shortens the 0xD6 state frame. The firmware's own C packer (compiled here) against the Python decoder."""
import ctypes
import math
import os
import random
import shutil
import subprocess
import tempfile
import unittest

from nimbus_pc import crsf as C

SRC = os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "firmware", "fc_matek_h743", "builds", "2026-11-26_fc-hybrid-v4.8_BUILT_NOT-FLASHED", "src")


@unittest.skipUnless(shutil.which("gcc") and os.path.isdir(SRC), "needs gcc and the v4.8 firmware sources")
class TestStateFrameV48(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory(); so = os.path.join(cls.tmp.name, "pack.so")
        subprocess.run(["gcc", "-shared", "-fPIC", "-I", SRC, os.path.join(SRC, "crsf_state_pack.c"), "-o", so], check=True)
        cls.pack = ctypes.CDLL(so).crsfStatePack
        u8, u16, i16 = ctypes.c_uint8, ctypes.c_uint16, ctypes.c_int16
        cls.pack.argtypes = [ctypes.POINTER(u8), u16, i16, i16, u8, ctypes.c_bool, u8, u8] + [i16] * 6; cls.pack.restype = u8

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def send(self, dist=523, fx=-16, fy=34, q=88, ekf=True, flow=1, hold=0, vx=-312, vy=145, vz=60, roll=-120, pitch=85, yaw=15700):
        buf = (ctypes.c_uint8 * 32)(); n = self.pack(buf, dist, fx, fy, q, ekf, flow, hold, vx, vy, vz, roll, pitch, yaw)
        raw = C.encode_frame(C.TYPE_STATE, bytes(buf[:n])); frames = C.FrameParser().feed(raw)
        self.assertEqual(len(frames), 1)
        name, d = C.decode_telemetry(frames[0]); self.assertEqual(name, "state_compact")
        return raw, d

    def test_it_is_20_bytes_on_the_wire(self):
        self.assertEqual(len(self.send()[0]), 20)

    def test_random_values_within_the_field_resolution(self):
        r = random.Random(1)
        for _ in range(2000):
            a = dict(dist=r.randint(0, 5999), fx=r.randint(-250, 250), fy=r.randint(-250, 250), q=r.randint(0, 255), ekf=r.random() < 0.5, flow=r.randint(0, 1),
                     hold=r.choice((0, 0x04, 0x08, 0x14, 0x24, 0x84)), vx=r.randint(-2047, 2047), vy=r.randint(-2047, 2047), vz=r.randint(-1270, 1270),
                     roll=r.randint(-11000, 11000), pitch=r.randint(-11000, 11000), yaw=r.randint(-31415, 31415))
            d = self.send(**a)[1]
            self.assertEqual((d["dist_raw"], d["vx_mm"], d["vy_mm"], d["yaw_e4"], d["flow_quality"], d["ekf_valid"], d["flow_status"]),
                             (a["dist"], a["vx"], a["vy"], a["yaw"], a["q"], a["ekf"], a["flow"]))
            self.assertEqual((d["hold_active"], d["hold_fault"], d["hold_mode"], d["xy_disabled"], d["vel_body"]),
                             (bool(a["hold"] & 4), bool(a["hold"] & 8), (a["hold"] >> 4) & 3, bool(a["hold"] & 0x80), True))
            self.assertLessEqual(abs(d["vz_mm"] - a["vz"]), 5)
            self.assertLessEqual(abs(d["roll_e4"] - a["roll"]), 45); self.assertLessEqual(abs(d["pitch_e4"] - a["pitch"]), 45)      # 0.25 deg
            self.assertLessEqual(abs(d["flow_x_raw"] - a["fx"]), 2); self.assertLessEqual(abs(d["flow_y_raw"] - a["fy"]), 2)

    def test_out_of_range_saturates_and_never_wraps(self):
        d = self.send(vx=9000, vy=-9000, vz=30000, roll=31000, pitch=-31000, fx=900, fy=-900)[1]
        self.assertEqual((d["vx_mm"], d["vy_mm"], d["vz_mm"], d["flow_x_raw"], d["flow_y_raw"]), (2047, -2047, 1270, 254, -254))
        self.assertGreater(d["roll_e4"], 11000); self.assertLess(d["pitch_e4"], -11000)

    def test_flows_into_the_same_messages_as_before(self):
        s = dict(C.split_state(self.send()[1]))
        self.assertAlmostEqual(s["optrange"]["distance_m"], 0.523); self.assertAlmostEqual(s["kinematic"]["vx_mps"], -0.312)
        self.assertTrue(math.isnan(s["kinematic"]["x_m"]))
