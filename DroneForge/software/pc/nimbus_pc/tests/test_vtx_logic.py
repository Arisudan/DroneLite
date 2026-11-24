import unittest

from nimbus_pc import commands, vtx


class TestVtxLogic(unittest.TestCase):
    def test_request_bytes(self):
        self.assertEqual(vtx.request_config_frame().hex()[:24], "c80b7ac8ea500058000000" + "58")
        self.assertEqual(len(vtx.request_config_frame()), 13)
        self.assertEqual(vtx.request_config_frame()[:8].hex(), "c80b7ac8ea500058")

    def test_set_matches_nimbus_vector(self):
        self.assertEqual([f.hex() for f in vtx.set_frames("F", 4, 2)], ["c80c7cc8ea5000590004001b0006", "c8087cc8ea41020044d7"])

    def test_pit_mode_flag(self):
        fr = vtx.set_frames("F", 4, 2, pit_mode=True)
        asm = commands.MspResponseAssembler(commands.TYPE_MSP_WRITE)
        from nimbus_pc import crsf
        out = None
        for f in fr:
            for x in crsf.FrameParser().feed(f):
                out = asm.feed(x) or out
        self.assertTrue(commands.decode_vtx_set_config(out["payload"])["pit_mode"])

    def test_scan_and_best(self):
        p = bytes(range(48))
        rows = vtx.parse_scan(p)
        self.assertEqual(len(rows), 48)
        self.assertEqual(rows[0], ("A", 1, 5865, 0))
        self.assertEqual(vtx.best_channel(p), ("L", 8, 5621, 47))
        self.assertIsNone(vtx.parse_scan(b"\x00" * 47))
        self.assertIsNone(vtx.best_channel(bytes(48)))

    def test_power_choices(self):
        self.assertEqual(vtx.power_choices()[1], (2, "100 mW"))
        self.assertEqual(vtx.power_choices(3)[2], (3, "level 3"))

    def test_parse_config(self):
        from nimbus_pc.emulator import VTX_CONFIG
        d = vtx.parse_config(VTX_CONFIG)
        self.assertEqual((d["band_letter"], d["channel"], d["power_level"]), ("F", 2, 2))


if __name__ == "__main__":
    unittest.main()
