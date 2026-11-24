import math
import os
import struct
import time
import unittest

from nimbus_pc import crsf
from nimbus_pc.emulator import DongleEmulator
from nimbus_pc.session import Session


def dec(t, payload):
    return crsf.decode_telemetry(crsf.Frame(0xC8, t, payload))


class TestIdentifiedCustomFrames(unittest.TestCase):
    def test_rangefinder_0xd0(self):
        name, d = dec(crsf.TYPE_RANGEFINDER, struct.pack(">HH", 150, 900))
        self.assertEqual((name, d["distance_m"], d["strength"]), ("rangefinder", 1.5, 900))
        self.assertIsNone(dec(crsf.TYPE_RANGEFINDER, b"\x00\x01"))

    def test_drone_linked_0xe2(self):
        self.assertTrue(dec(crsf.TYPE_DRONE_LINKED, b"\x01")[1]["is_linked"])
        self.assertFalse(dec(crsf.TYPE_DRONE_LINKED, b"\x00")[1]["is_linked"])
        self.assertIsNone(dec(crsf.TYPE_DRONE_LINKED, b""))

    def test_fc_uid_0xe3(self):
        d = dec(crsf.TYPE_FC_UID, struct.pack("<3I", 0x12345678, 2, 3))[1]
        self.assertEqual((d["uid0"], d["uid_hex"]), (0x12345678, "12345678-00000002-00000003"))

    def test_magnetometer_0x12_scale(self):
        d = dec(crsf.TYPE_MAG, struct.pack(">3h", 30, -60, 90))[1]
        self.assertEqual((d["field_forward"], d["field_right"], d["field_down"]), (10.0, -20.0, 30.0))

    def test_raw_imu_0xd1_scale(self):
        d = dec(crsf.TYPE_RAW_IMU, struct.pack(">6h", 100, -100, 0, 0, 50, 981))[1]
        self.assertAlmostEqual(d["gyro_forward"], 180.0 / math.pi)
        self.assertAlmostEqual(d["gyro_right"], -180.0 / math.pi)
        self.assertAlmostEqual(d["accel_down"], 9.81)
        self.assertEqual(d["raw"], [100, -100, 0, 0, 50, 981])

    def test_vrx_top_channels_0xe1(self):
        d = dec(crsf.TYPE_VRX_TOP_CHANNELS, bytes(range(48)))[1]
        self.assertEqual((len(d["rssi_by_freq_order"]), d["rssi_by_freq_order"][47]), (48, 47))
        self.assertIsNone(dec(crsf.TYPE_VRX_TOP_CHANNELS, bytes(47)))

    def test_undecoded_custom_frames_are_logged_raw(self):
        for t in (crsf.TYPE_MIGHTY_CAM, crsf.TYPE_MOTOR_ERPM, crsf.TYPE_CUSTOM_E4):         # 0xD4 is decoded now (the FC's EKF state)
            name, d = dec(t, bytes([1, 2, 3, 4]))
            self.assertTrue(name.startswith("custom_"))
            self.assertEqual((d["decoded"], d["hex"]), (False, "01020304"))

    def test_unknown_type_still_ignored(self):
        self.assertIsNone(dec(0x99, b"abc"))


class TestEmulatorStreamsIdentifiedFrames(unittest.TestCase):
    def test_session_sees_new_frames(self):
        emu = DongleEmulator().start()
        try:
            with Session(emu.path, ping=False, listen_only=True) as s:
                time.sleep(1.3)
                t = s.telemetry
        finally:
            emu.stop()
        self.assertTrue(t["drone_linked"]["is_linked"])
        self.assertEqual(t["fc_uid"]["uid_hex"], "00000001-00000002-00000003")
        self.assertAlmostEqual(t["rangefinder"]["distance_m"], 1.5, places=1)


if __name__ == "__main__":
    unittest.main(verbosity=2)
