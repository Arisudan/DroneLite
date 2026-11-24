import os
import subprocess
import sys
import time
import unittest

from nimbus_pc import commands, crsf
from nimbus_pc.emulator import VTX_CONFIG, DongleEmulator
from nimbus_pc.session import Session

HOST_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


class TestVtxPowerFrames(unittest.TestCase):
    def test_exact_bytes_band_f_channel_4_power_2(self):
        frames = commands.encode_msp_set_vtx_config_power("F", 4, 2)
        # body = 00 | 59 00 | 04 00 | 1b 00 02 00 | xor(0x44); split 7 + 3, status 0x50 then 0x41
        self.assertEqual([f.hex() for f in frames], ["c80c7cc8ea5000590004001b0006", "c8087cc8ea41020044d7"])

    def test_frame_structure(self):
        frames = commands.encode_msp_set_vtx_config_power("A", 1, 1)
        self.assertEqual(len(frames), 2)
        for f in frames:
            self.assertEqual((f[0], f[2], f[3], f[4]), (0xC8, 0x7C, 0xC8, 0xEA))      # sync, MSP_WRITE, FC, radio
            self.assertEqual(f[1], len(f) - 2)
            self.assertEqual(f[-1], crsf.crc8(f[2:-1]))
        self.assertEqual((frames[0][5], frames[1][5]), (0x50, 0x41))                  # start+seq0, then seq1

    def test_xor_checksum_in_last_body_byte(self):
        for band, ch, pw in (("F", 4, 2), ("L", 8, 5), ("B", 3, 9)):
            frames = commands.encode_msp_set_vtx_config_power(band, ch, pw)
            body = frames[0][6:-1] + frames[1][6:-1]
            x = 0
            for b in body[:-1]:
                x ^= b
            self.assertEqual((len(body), body[-1]), (10, x))

    def test_betaflight_decoding_of_the_payload(self):
        # Betaflight: v <= 63 -> band = v/8 + 1, channel = v%8 + 1
        for band, idx in (("A", 1), ("B", 2), ("E", 3), ("F", 4), ("R", 5), ("L", 6)):
            for ch in (1, 8):
                frames = commands.encode_msp_set_vtx_config_power(band, ch, 3)
                asm = commands.MspResponseAssembler(commands.TYPE_MSP_WRITE)
                out = None
                for fr in frames:
                    for f in crsf.FrameParser().feed(fr):
                        out = asm.feed(f) or out
                self.assertEqual((out["cmd"], len(out["payload"])), (commands.MSP_SET_VTX_CONFIG, 4))
                d = commands.decode_vtx_set_config(out["payload"])
                self.assertEqual((d["band_index"], d["band_letter"], d["channel"], d["power_level"], d["pit_mode"]),
                                 (idx, band, ch, 3, False))

    def test_validation_matches_nimbus(self):
        for args in (("Z", 1, 1), ("A", 0, 1), ("A", 9, 1), ("A", 1, 0), ("A", 1, 256)):
            with self.assertRaises(ValueError):
                commands.encode_msp_set_vtx_config_power(*args)

    def test_vtx_config_reply_decoder(self):
        d = commands.decode_vtx_config(VTX_CONFIG)
        self.assertEqual((d["vtx_type"], d["band"], d["channel"], d["power_level"], d["frequency_mhz"]), (3, 4, 2, 2, 5800))
        self.assertEqual((d["table_bands"], d["table_channels"], d["table_power_levels"], d["device_ready"]), (6, 8, 5, True))
        self.assertIsNone(commands.decode_vtx_config(bytes(14)))


class TestVtxEndToEnd(unittest.TestCase):
    def setUp(self):
        self.emu = DongleEmulator().start()

    def tearDown(self):
        self.emu.stop()

    def run_cli(self, *a):
        return subprocess.run([sys.executable, "-m", "nimbus_pc", *a], cwd=HOST_DIR, capture_output=True, text=True, timeout=30)

    def test_cli_set_reaches_the_dongle(self):
        r = self.run_cli("vtx", "--port", self.emu.path, "set", "--band", "F", "--channel", "4", "--power", "2")
        self.assertEqual(r.returncode, 0, r.stderr)
        time.sleep(0.3)
        self.assertEqual(self.emu.vtx_writes, [{"band_index": 4, "band_letter": "F", "channel": 4, "power_level": 2, "pit_mode": False}])
        self.assertIn("2 CRSF frames", r.stdout)

    def test_cli_get_decodes_reply(self):
        r = self.run_cli("vtx", "--port", self.emu.path, "get")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("5800", r.stdout)
        self.assertIn("'table_power_levels': 5", r.stdout)

    def test_cli_rejects_bad_arguments(self):
        r = self.run_cli("vtx", "--port", self.emu.path, "set", "--band", "F", "--channel", "9", "--power", "2")
        self.assertNotEqual(r.returncode, 0)
        self.assertEqual(self.emu.vtx_writes, [])


if __name__ == "__main__":
    unittest.main(verbosity=2)
