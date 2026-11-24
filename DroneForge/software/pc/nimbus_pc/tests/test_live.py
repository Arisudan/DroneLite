import io
import unittest

from nimbus_pc import crsf, live
from nimbus_pc.emulator import DongleEmulator
from nimbus_pc.sender import Controls
from nimbus_pc.session import Session


class FakeClock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def frame(t, payload=b"\x00" * 10):
    return crsf.Frame(sync=0xC8, type=t, payload=payload)


class TestLiveState(unittest.TestCase):
    def test_rates_ages_and_counts(self):
        clk = FakeClock(); st = live.LiveState(clk)
        for _ in range(10):
            st.note_in(frame(0xD3)); clk.t += 0.1
        self.assertEqual(st.in_count[0xD3], 10)
        self.assertGreater(st.rate(st.in_times, 0xD3), 1.5)
        st.note_decoded("optrange", {"distance_m": 0.5, "dist_strength": 200, "flow_quality": 30, "flow_x_raw": 1, "flow_y_raw": 2,
                                     "velocity_forward": 0.01, "velocity_right": 0.0})
        clk.t += 2.0
        self.assertAlmostEqual(st.age("optrange"), 2.0, places=3)

    def test_outgoing_frames_are_counted_by_type(self):
        st = live.LiveState(FakeClock())
        st.note_out(crsf.encode_rc_frame([992] * 16) * 3 + crsf.encode_device_ping(0xEE))
        self.assertEqual(st.out_count[0x16], 3)
        self.assertEqual(st.out_count[0x28], 1)

    def test_events_on_changes(self):
        st = live.LiveState(FakeClock())
        st.note_decoded("flight_mode", {"mode": "ACRO"}); st.note_decoded("flight_mode", {"mode": "STAB*"})
        st.note_decoded("link", {"up_lq": 100}); st.note_decoded("link", {"up_lq": 0})
        texts = [t for _, t in st.events]
        self.assertIn("flight mode -> STAB*", texts)
        self.assertIn("radio link LOST", texts)


class TestRfTiming(unittest.TestCase):
    def test_radio_id_sync_frame_is_decoded(self):
        st = live.LiveState(FakeClock())
        st.note_in(frame(0x3A, bytes.fromhex('eaee1000009c40ffffeade')))        # seen on the real TX: 40000 x 0.1 us = 4000 us
        self.assertIn('4000 us = 250 Hz', live.rf_timing(st))
        self.assertIn('RADIO_ID_SYNC', live.render(st, color=False))


class TestWarnings(unittest.TestCase):
    def test_banner_when_no_data(self):
        clk = FakeClock(); st = live.LiveState(clk)
        st.note_in(frame(0x14)); clk.t += 6.0
        self.assertIn('NO DATA from the dongle', live.render(st, color=False))
        st.note_in(frame(0x14))
        self.assertNotIn('NO DATA', live.render(st, color=False))


class TestRender(unittest.TestCase):
    def setUp(self):
        self.clk = FakeClock(); self.st = live.LiveState(self.clk)

    def test_lists_every_nimbus_group_with_dashes_when_empty(self):
        text = live.render(self.st, color=False)
        for g in live.GROUPS:
            self.assertIn(g, text)
        self.assertIn("COMMANDS OUT", text)

    def test_shows_decoded_values_and_raw_for_undecoded(self):
        self.st.note_decoded("battery", {"voltage_v": 16.16, "current_a": 0.0, "capacity_mah": 0, "remaining_pct": 0})
        self.st.note_in(frame(0xD5, bytes.fromhex("ffc6000c0025")))
        text = live.render(self.st, color=False)
        self.assertIn("16.16 V", text)
        self.assertIn("raw ffc6000c0025", text)
        self.assertIn("MOTOR_ERPM*", text)

    def test_raw_section_lists_both_directions(self):
        self.st.note_in(frame(0xD3)); self.st.note_out(crsf.encode_device_ping(0xEE))
        text = live.render(self.st, color=False, raw_lines=5)
        self.assertIn("<- 0xD3", text)
        self.assertIn("-> 0x28", text)


class TestLiveRunWithEmulator(unittest.TestCase):
    def test_full_run_shows_both_directions(self):
        emu = DongleEmulator().start()
        try:
            st = live.LiveState()
            sess = Session(emu.path, hz=100, allow_arm=False, ping=True, on_telemetry=st.note_decoded, on_frame=st.note_in)
            sess.link.on_write = st.note_out
            sess.start()
            try:
                out = io.StringIO()
                last = live.run(sess, st, port=emu.path, duration=2.5, source="idle", set_controls=lambda: sess.set_controls(Controls()),
                                stream=out, clear=False, color=False, fps=10, raw_lines=4)
            finally:
                sess.stop()
        finally:
            emu.stop()
        self.assertIn("linkStats", last)
        self.assertRegex(last, r"optrange\s+dist \d+\.\d+ m")
        self.assertIn("RC_CHANNELS", last)            # OUT table
        self.assertIn("DEVICE_INFO", last)
        self.assertIn("ExpressLRS", last)   # device names from the emulator's pings
        self.assertGreater(st.out_count.get(0x16, 0), 100)


if __name__ == "__main__":
    unittest.main()
