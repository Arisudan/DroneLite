import os
import struct
import subprocess
import sys
import time
import unittest

from nimbus_pc import commands, crsf
from nimbus_pc.emulator import BOARD_INFO, DongleEmulator, _chunk_msp_response
from nimbus_pc.gamepad import JS_EVENT, GamepadInput
from nimbus_pc.mapper import throttle01_to_crsf
from nimbus_pc.sender import Controls
from nimbus_pc.session import Session

HOST_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def frames_of(raw: bytes):
    return crsf.FrameParser().feed(raw)


class TestBatteryUnits(unittest.TestCase):
    def test_spec_units_and_centivolts(self):
        spec = crsf.decode_battery(struct.pack('>HH', 162, 0) + bytes(4))              # 16.2 V in 0.1 V units
        centi = crsf.decode_battery(struct.pack('>HH', 1616, 0) + bytes(4))            # same pack in 0.01 V units (this FC)
        self.assertAlmostEqual(spec['voltage_v'], 16.2, places=2)
        self.assertAlmostEqual(centi['voltage_v'], 16.16, places=2)


class TestCommandFrames(unittest.TestCase):
    def test_bind_frame_exact(self):
        # EE 07 32 EE EA 10 01 00 CRC  (command 0x10 / sub 0x01, no payload, 0 command-CRC slot)
        f = commands.encode_receiver_bind()
        self.assertEqual(f.hex(), "ee0732eeea10010047")
        self.assertEqual(len(f), 9)

    def test_command_frame_layout_and_crc(self):
        f = commands.encode_command_frame(0x80, 0x41, bytes([0x18, 4]))
        self.assertEqual((f[0], f[1], f[2], f[3], f[4], f[5], f[6]), (0xEE, 9, 0x32, 0xEE, 0xEA, 0x80, 0x41))
        self.assertEqual(f[-2], 0)                                   # command-CRC slot is zero in Nimbus
        self.assertEqual(f[-1], crsf.crc8(f[2:-1]))
        parsed = frames_of(f)
        self.assertEqual((len(parsed), parsed[0].type), (1, commands.TYPE_COMMAND))

    def test_set_bind_uid_and_vrx(self):
        uid = bytes(range(6))
        self.assertEqual(commands.encode_set_bind_uid(uid)[5:7], bytes([0x80, 0x42]))
        self.assertEqual(commands.encode_set_bind_uid(uid)[7:13], uid)
        with self.assertRaises(ValueError):
            commands.encode_set_bind_uid(b"12345")
        self.assertEqual(commands.encode_vrx_scan()[5:8], bytes([0x80, 0x40, 0x01]))
        self.assertEqual(commands.encode_vrx_connect("F", 4)[5:9], bytes([0x80, 0x41, 24, 4]))   # F = index 3 -> code 24
        for bad in (0, 9):
            with self.assertRaises(ValueError):
                commands.encode_vrx_connect("A", bad)
        with self.assertRaises(ValueError):
            commands.encode_vrx_connect("Z", 1)
        self.assertEqual([commands.band_code(c) for c in "ABEFRL"], [0, 8, 16, 24, 32, 40])

    def test_uid_from_phrase_is_deterministic_6_bytes(self):
        a = commands.uid_from_phrase("hello")
        self.assertEqual((len(a), a), (6, commands.uid_from_phrase("hello")))
        self.assertNotEqual(a, commands.uid_from_phrase("hello2"))

    def test_msp_request_exact(self):
        f = commands.encode_msp_board_info_request()
        self.assertEqual(f.hex(), "c80b7ac8ea5000040000000462")      # C8 0B 7A C8 EA 50 00 04 00 00 00 xor crc
        self.assertEqual(commands.encode_msp_vtx_config_request()[7], 0x58)
        self.assertEqual(len(f), 13)
        self.assertEqual(frames_of(f)[0].type, commands.TYPE_MSP_REQ)
        with self.assertRaises(ValueError):
            commands.encode_msp_request(0x100)


class TestMspAssembler(unittest.TestCase):
    def test_single_and_multi_chunk(self):
        for payload in (b"", b"abc", bytes(range(60))):
            asm = commands.MspResponseAssembler()
            out = None
            for fr in _chunk_msp_response(0x58, payload, crsf.ADDR_RADIO, crsf.ADDR_FC, chunk=20):
                for f in frames_of(fr):
                    out = asm.feed(f) or out
            self.assertEqual((out["cmd"], out["payload"]), (0x58, payload))

    def test_wrong_sequence_is_dropped(self):
        asm = commands.MspResponseAssembler()
        frs = [frames_of(x)[0] for x in _chunk_msp_response(0x04, bytes(60), crsf.ADDR_RADIO, crsf.ADDR_FC, chunk=20)]
        self.assertIsNone(asm.feed(frs[0]))
        self.assertIsNone(asm.feed(frs[2]))          # skipped chunk 1 -> out of sequence
        self.assertEqual(asm.dropped, 1)
        self.assertIsNone(asm.feed(frs[1]))          # state was reset, so a stray continuation is ignored too

    def test_board_info_decode(self):
        d = commands.decode_board_info(BOARD_INFO)
        self.assertEqual((d["board_id"], d["mcu"], d["target"], d["manufacturer"]), ("SH74", "STM32H743", "MATEKH743", "MTKS"))


class TestGamepad(unittest.TestCase):
    def setUp(self):
        self.r, self.w = os.pipe()
        self.pad = GamepadInput(fd=self.r)

    def tearDown(self):
        os.close(self.w)
        self.pad.close()

    def push(self, values, init=False):
        for i, v in enumerate(values):
            os.write(self.w, JS_EVENT.pack(0, v, 0x82 if init else 0x02, i))
        self.pad.poll()

    def test_not_ready_is_safe(self):
        self.push([0, 0, -32767], init=True)        # only 3 of 7 mapped axes reported
        c = self.pad.controls()
        self.assertFalse(self.pad.ready)
        self.assertEqual((c.throttle, c.arm), (0.0, False))

    def test_mapping_and_scaling(self):
        self.push([32767, -32767, 0, 16384, -32767, 0, 32767], init=True)
        c = self.pad.controls()
        self.assertAlmostEqual(c.roll, 1.0); self.assertAlmostEqual(c.pitch, -1.0)
        self.assertAlmostEqual(c.throttle, 0.5, places=3); self.assertAlmostEqual(c.yaw, 0.5, places=3)
        self.assertFalse(c.arm)
        self.assertEqual((c.aux2, c.aux3), (992, 1811))

    def test_arm_switch_already_on_at_start_never_arms(self):
        # found on the real Boxer: starting with the arm switch ON and throttle low must not arm
        self.push([0, 0, -32767, 0, 32767, -32767, -32767], init=True)
        c = self.pad.controls()
        self.assertFalse(c.arm)
        self.assertTrue(self.pad.arm_blocked)
        self.push([0, 0, -32767, 0, 32767])
        self.assertFalse(self.pad.controls().arm)            # still ON: still blocked
        self.push([0, 0, -32767, 0, -32767])
        self.assertFalse(self.pad.controls().arm)            # cycled OFF
        self.assertFalse(self.pad.arm_blocked)
        self.push([0, 0, -32767, 0, 32767])
        self.assertTrue(self.pad.controls().arm)             # now allowed

    def test_arm_requires_low_throttle_at_switch(self):
        self.push([0, 0, -32767, 0, -32767, -32767, -32767], init=True)
        self.assertFalse(self.pad.controls().arm)
        self.push([0, 0, -32767, 0, 32767])          # switch up with throttle low: allowed
        self.assertTrue(self.pad.controls().arm)
        self.push([0, 0, 0, 0, -32767])               # disarm, then raise throttle and arm again: blocked
        self.assertFalse(self.pad.controls().arm)
        self.push([0, 0, 0, 0, 32767])
        c = self.pad.controls()
        self.assertFalse(c.arm); self.assertTrue(self.pad.arm_blocked)
        self.push([0, 0, 0, 0, -32767]); self.pad.controls()            # cycle the switch off
        self.assertFalse(self.pad.arm_blocked)

    def test_device_loss_returns_safe(self):
        self.push([0, 0, 0, 0, 32767, 0, 0], init=True)
        os.close(self.w)
        self.pad.poll()
        self.assertFalse(self.pad.connected)
        c = self.pad.controls()
        self.assertEqual((c.throttle, c.arm), (0.0, False))
        self.w = os.open(os.devnull, os.O_WRONLY)       # so tearDown can close something


class TestEndToEndWithEmulator(unittest.TestCase):
    def setUp(self):
        self.emu = DongleEmulator().start()

    def tearDown(self):
        self.emu.stop()

    def test_pings_answered_and_telemetry_flows(self):
        with Session(self.emu.path, ping=True) as s:
            s.set_controls(Controls())
            time.sleep(1.4)
            t = s.telemetry
        for key in ("device_info_0xee", "device_info_0xec", "device_info_0xc8", "link", "battery", "attitude",
                    "flight_mode", "optrange"):
            self.assertIn(key, t)
        self.assertEqual(sorted(set(self.emu.pings)), [0xC8, 0xEC, 0xEE])
        self.assertLessEqual(len(self.emu.pings), 4)                # each target pinged once, then answered
        self.assertEqual(t["device_info_0xee"]["name"], "ExpressLRS TX")
        self.assertEqual(t["link"]["up_lq"], 98)
        self.assertAlmostEqual(t["battery"]["voltage_v"], 25.5)
        self.assertGreater(t["optrange"]["distance_m"], 1.4)

    def test_msp_board_info_roundtrip(self):
        with Session(self.emu.path, listen_only=True, ping=False) as s:
            r = s.request_msp(commands.MSP_BOARD_INFO, timeout=3.0)
        self.assertEqual(commands.decode_board_info(r["payload"])["target"], "MATEKH743")
        self.assertEqual(self.emu.msp_requests, [commands.MSP_BOARD_INFO])

    def test_commands_reach_the_dongle(self):
        with Session(self.emu.path, listen_only=True, ping=False) as s:
            s.bind(phrase="abc")
            s.send_frame(commands.encode_vrx_scan())
            time.sleep(0.3)
        kinds = [(c[3], c[4]) for c in self.emu.commands]      # c = type, dest, origin, cmd, sub, ...
        self.assertEqual(kinds, [(0x80, 0x42), (0x10, 0x01), (0x80, 0x40)])

    def _armed_throttle_run(self, seconds):
        s = Session(self.emu.path, hz=100, allow_arm=True, ping=False, watch_optrange=True)
        s.start()
        trip = None
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            s.set_controls(Controls(throttle=0.5, arm=True))
            if trip is None and s.watchdog.forcing_zero:
                trip = time.monotonic() - t0
            time.sleep(0.02)
        thr = self.emu.last_channels[2]
        s.stop()
        return trip, thr

    def test_watchdog_quiet_with_live_sensor(self):
        trip, thr = self._armed_throttle_run(1.6)
        self.assertIsNone(trip)
        self.assertEqual(thr, throttle01_to_crsf(0.5))

    def test_watchdog_trips_when_sensor_dies(self):
        self.emu.optrange_enabled = False
        trip, thr = self._armed_throttle_run(1.6)
        print(f"\n  sensor silent: throttle forced to zero after {trip:.2f} s")
        self.assertTrue(trip is not None and 0.4 <= trip <= 0.9, trip)
        self.assertEqual(thr, 172)

    def test_watchdog_trips_on_flatline(self):
        self.emu.optrange_frozen = True
        trip, thr = self._armed_throttle_run(2.0)
        print(f"\n  sensor frozen: throttle forced to zero after {trip:.2f} s")
        self.assertTrue(trip is not None and 0.4 <= trip <= 1.2, trip)
        self.assertEqual(thr, 172)

    def test_cli_smoke(self):
        def run(*a):
            return subprocess.run([sys.executable, "-m", "nimbus_pc", *a], cwd=HOST_DIR, capture_output=True,
                                  text=True, timeout=30)
        r = run("send", "--port", self.emu.path, "--duration", "1.3")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("sent ", r.stdout)
        r = run("msp", "--port", self.emu.path, "board-info")
        self.assertIn("MATEKH743", r.stdout)
        r = run("send", "--port", "/dev/does-not-exist", "--duration", "1")
        self.assertEqual(r.returncode, 2)
        self.assertIn("cannot open", r.stderr)
        r = run("vrx", "--port", self.emu.path, "connect", "--band", "B", "--channel", "9")
        self.assertNotEqual(r.returncode, 0)                          # channel validation


if __name__ == "__main__":
    unittest.main(verbosity=2)
