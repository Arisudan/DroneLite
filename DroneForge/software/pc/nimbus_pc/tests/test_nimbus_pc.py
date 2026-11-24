import os
import statistics
import struct
import time
import unittest

from nimbus_pc import crsf
from nimbus_pc.mapper import _lroundf, crsf_to_throttle01, stick_to_crsf, throttle01_to_crsf
from nimbus_pc.sender import Controls, CrsfSender, SerialLink, TelemetryReader
from nimbus_pc.watchdog import FLATLINE, MISSING, OK, SensorWatchdog

# Idle frame measured/derived from NimbusOS (sticks centred, throttle min, AUX1-3 at 172)
NIMBUS_IDLE_FRAME = "c81816e0031f2bc0c70a56b0027ce0031ff8c0073ef0810f7c8a"


def make_pty():
    master, slave = os.openpty()
    return master, slave, os.ttyname(slave)


def read_available(master, seconds):
    import select
    data, end = b"", time.monotonic() + seconds
    while time.monotonic() < end:
        r, _, _ = select.select([master], [], [], 0.02)
        if r:
            data += os.read(master, 4096)
    return data


class TestCrsf(unittest.TestCase):
    def test_crc_check_value(self):
        self.assertEqual(crsf.crc8(b"123456789"), 0xBC)   # CRC-8/DVB-S2 standard check value

    def test_idle_frame_matches_nimbus(self):
        ch = Controls().to_channels(allow_arm=False)
        self.assertEqual((ch[8], ch[9]), (172, 172))       # the ONLY deliberate differences: CH9 = FC height-hold request, CH10 = hover-reference throttle for it; both minimum (off) by default, Nimbus leaves them at mid
        ch[8] = ch[9] = 992
        self.assertEqual(crsf.encode_rc_frame(ch).hex(), NIMBUS_IDLE_FRAME)

    def test_flight_mode_text_carries_motors_and_keeps_its_old_meaning(self):
        d = crsf.decode_flight_mode(b"STAB HH M1523,1524,1525,1526 T1613*\0")
        self.assertEqual((d["mode"], d["motors"], d["thr_us"]), ("STAB HH*", [1523, 1524, 1525, 1526], 1613))      # '*' (disarmed) and ' HH' still readable as before
        self.assertEqual(crsf.decode_flight_mode(b"STAB*\0"), {"mode": "STAB*"})                                     # old firmware: unchanged
        self.assertEqual(crsf.decode_flight_mode(b"STAB M1000,1000,1000,1000 T1000\0")["mode"], "STAB")            # armed: no star

    def test_flight_mode_text_diagnostic_variants(self):
        d = crsf.decode_flight_mode(b"STAB P1,-2,3 I-120,300,12 D7,8,-9*\0")
        self.assertEqual((d["mode"], d["pid_i"], d["pid_p"], d["pid_d"]), ("STAB*", [-120, 300, 12], [1, -2, 3], [7, 8, -9]))
        d = crsf.decode_flight_mode(b"STAB HH G-12,5,1 S-400,500,2\0")
        self.assertEqual((d["mode"], d["gyro_dps10"], d["pid_sum"]), ("STAB HH", [-12, 5, 1], [-400, 500, 2]))

    def test_channel_roundtrip(self):
        ch = [172, 992, 1811, 500, 0, 2047, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
        self.assertEqual(crsf.unpack_channels(crsf.pack_channels(ch)), ch)

    def test_frame_layout(self):
        f = crsf.encode_rc_frame([992] * 16)
        self.assertEqual((len(f), f[0], f[1], f[2]), (26, 0xC8, 0x18, 0x16))

    def test_parser_resync_and_crc(self):
        good = crsf.encode_rc_frame([992] * 16)
        bad = bytearray(good); bad[10] ^= 0xFF
        stream = b"\x00\x13garbage" + good + bytes(bad) + good + b"\xc8"
        p = crsf.FrameParser()
        frames = p.feed(stream[:30]) + p.feed(stream[30:])
        self.assertEqual(len(frames), 2)
        self.assertGreaterEqual(p.crc_errors, 1)
        self.assertEqual(crsf.unpack_channels(frames[0].payload), [992] * 16)

    def test_mapper(self):
        self.assertEqual((stick_to_crsf(0), stick_to_crsf(-1), stick_to_crsf(1), stick_to_crsf(9)), (992, 172, 1811, 1811))
        self.assertEqual((throttle01_to_crsf(0), throttle01_to_crsf(1), throttle01_to_crsf(-1)), (172, 1811, 172))
        self.assertAlmostEqual(crsf_to_throttle01(throttle01_to_crsf(0.5)), 0.5, places=3)

    def test_c_rounding(self):
        self.assertEqual((_lroundf(992.5), _lroundf(-0.5), _lroundf(991.4)), (993, -1, 991))

    def test_telemetry_decoders(self):
        link = crsf.Frame(0xEA, crsf.TYPE_LINK_STATISTICS, bytes([40, 45, 98, 0xFB, 0, 4, 3, 38, 97, 10]))
        name, d = crsf.decode_telemetry(link)
        self.assertEqual((name, d["up_rssi1_dbm"], d["up_lq"], d["up_snr_db"], d["down_lq"]), ("link", -40, 98, -5, 97))
        batt = crsf.Frame(0xC8, crsf.TYPE_BATTERY, struct.pack(">HH", 255, 120) + (1500).to_bytes(3, "big") + bytes([80]))
        name, d = crsf.decode_telemetry(batt)
        self.assertEqual((d["voltage_v"], d["current_a"], d["capacity_mah"], d["remaining_pct"]), (25.5, 12.0, 1500, 80))
        att = crsf.Frame(0xC8, crsf.TYPE_ATTITUDE, struct.pack(">hhh", 1000, -2000, 15708))
        d = crsf.decode_telemetry(att)[1]
        self.assertAlmostEqual(d["pitch_rad"], 0.1); self.assertAlmostEqual(d["roll_rad"], -0.2); self.assertAlmostEqual(d["yaw_rad"], 1.5708)
        fm = crsf.Frame(0xC8, crsf.TYPE_FLIGHT_MODE, b"ANGL\x00")
        self.assertEqual(crsf.decode_telemetry(fm)[1]["mode"], "ANGL")
        self.assertIsNone(crsf.decode_telemetry(crsf.Frame(0xC8, 0x99, b"")))


class TestSender(unittest.TestCase):
    def setUp(self):
        self.master, self.slave, self.path = make_pty()
        self.link = SerialLink(self.path)
        self.link.open()

    def tearDown(self):
        self.link.close()
        os.close(self.slave)
        os.close(self.master)

    def test_rate_and_validity(self):
        s = CrsfSender(self.link, hz=100.0)
        s.start(); s.set_controls(Controls())
        t0 = time.monotonic(); stamps = []
        import select
        parser = crsf.FrameParser(); frames = []
        while time.monotonic() - t0 < 3.0:
            s.set_controls(Controls())
            r, _, _ = select.select([self.master], [], [], 0.01)
            if r:
                now = time.monotonic()
                for f in parser.feed(os.read(self.master, 4096)):
                    frames.append(f); stamps.append(now)
        s.stop()
        self.assertEqual(parser.crc_errors, 0)
        frames = [f for f in frames if f.type == crsf.TYPE_RC_CHANNELS]          # pings share the stream
        self.assertTrue(all(len(f.payload) == 22 for f in frames))
        rate = len(frames) / 3.0
        print(f"\n  measured rate {rate:.1f} Hz over 3 s, {len(frames)} RC frames, sender max interval {s.max_interval_s*1000:.1f} ms")
        self.assertTrue(95 <= rate <= 105, rate)

    def test_stale_input_failsafe(self):
        s = CrsfSender(self.link, hz=100.0, stale_after_s=0.25)
        s.start()
        s.set_controls(Controls(throttle=1.0, roll=1.0))
        time.sleep(0.3)   # no more updates -> input goes stale at 0.25 s
        data = read_available(self.master, 0.6)
        s.stop()
        chans = [crsf.unpack_channels(f.payload) for f in crsf.FrameParser().feed(data) if f.type == crsf.TYPE_RC_CHANNELS]
        thr = [c[2] for c in chans]
        self.assertIn(1811, thr)       # fresh input was sent first
        self.assertEqual(thr[-1], 172)  # then failsafe: throttle minimum
        self.assertEqual(chans[-1][0], 992)   # sticks centred

    def test_arm_gating(self):
        self.assertEqual(Controls(arm=True).to_channels(allow_arm=False)[4], 172)
        self.assertEqual(Controls(arm=True).to_channels(allow_arm=True)[4], 1811)
        self.assertEqual(Controls(arm=False).to_channels(allow_arm=True)[4], 172)

    def test_stop_sends_safe_frame(self):
        s = CrsfSender(self.link, hz=50.0, allow_arm=True)
        s.start(); s.set_controls(Controls(throttle=1.0, arm=True)); time.sleep(0.15)
        s.stop()
        frames = [f for f in crsf.FrameParser().feed(read_available(self.master, 0.2)) if f.type == crsf.TYPE_RC_CHANNELS]
        last = crsf.unpack_channels(frames[-1].payload)
        self.assertEqual((last[2], last[4]), (172, 172))   # throttle min, disarmed

    def test_telemetry_reader_over_pty(self):
        batt = crsf.encode_frame(crsf.TYPE_BATTERY, struct.pack(">HH", 252, 0) + bytes(3) + bytes([100]), sync=0xC8)
        r = TelemetryReader(self.link); r.start()
        os.write(self.master, b"\x00" + batt + batt)
        time.sleep(0.3); r.stop()
        self.assertEqual(r.latest["battery"]["voltage_v"], 25.2)
        self.assertEqual(r.frame_count, 2)


class TestPingAndDeviceInfo(unittest.TestCase):
    def test_ping_frame_bytes(self):
        # EE 04 28 <dest> <origin=EA> CRC, layout from EncodeDevicePingFrame
        self.assertEqual(crsf.encode_device_ping(crsf.ADDR_TX).hex(), "ee0428eeea97")
        for dest in (crsf.ADDR_TX, crsf.ADDR_RX, crsf.ADDR_FC):
            f = crsf.FrameParser().feed(crsf.encode_device_ping(dest))
            self.assertEqual((len(f), f[0].type, f[0].payload), (1, crsf.TYPE_DEVICE_PING, bytes([dest, 0xEA])))

    def test_device_info_decode(self):
        payload = bytes([0xEA, 0xEE]) + b"ExpressLRS TX\x00" + b"ELRS" + bytes(4) + bytes([3, 5, 3, 0]) + bytes([10, 0])
        d = crsf.decode_telemetry(crsf.Frame(0xEE, crsf.TYPE_DEVICE_INFO, payload))[1]
        self.assertEqual((d["origin"], d["name"], d["version"], d["param_count"]), (0xEE, "ExpressLRS TX", "3.5.3", 10))

    def test_device_info_reader_keys_per_origin(self):
        master, slave, path = make_pty()
        link = SerialLink(path); link.open()
        try:
            r = TelemetryReader(link); r.start()
            info = crsf.encode_frame(crsf.TYPE_DEVICE_INFO, bytes([0xEA, 0xEC]) + b"RX\x00" + bytes(12) + bytes([0, 0]), sync=0xEC)
            os.write(master, info); time.sleep(0.3); r.stop()
            self.assertIn("device_info_0xec", r.latest)
        finally:
            link.close(); os.close(slave); os.close(master)


class FakeClockWatchdogTests(unittest.TestCase):
    """Rules ported from OptrangeFailsafe::Update: 5% throttle, 0.5 s missing, 0.5 s flatline, latch until disarm."""

    def test_inactive_and_low_throttle_never_trip(self):
        w = SensorWatchdog()
        self.assertEqual(w.update(False, 1.0, 100.0), OK)
        for t in range(0, 50):
            self.assertEqual(w.update(True, 0.02, t * 0.1), OK)   # armed but below 5%: never engages
        self.assertFalse(w.engaged)

    def test_missing_frames_after_half_second(self):
        w = SensorWatchdog()
        self.assertEqual(w.update(True, 0.5, 10.0), OK)           # engaged at t=10
        self.assertEqual(w.update(True, 0.5, 10.49), OK)
        self.assertEqual(w.update(True, 0.5, 10.50), MISSING)
        self.assertTrue(w.forcing_zero)
        self.assertEqual(w.update(True, 0.5, 11.0), OK)           # latched: trip reported only once
        self.assertTrue(w.forcing_zero)

    def test_changing_samples_keep_it_alive(self):
        w = SensorWatchdog()
        t = 0.0
        for i in range(60):
            w.feed(bytes([i % 256]), t)
            self.assertEqual(w.update(True, 0.5, t), OK)
            t += 0.1
        self.assertFalse(w.forcing_zero)

    def test_flatline_after_half_second_identical(self):
        w = SensorWatchdog()
        results = []
        t = 0.0
        for i in range(10):
            w.feed(b"\x01\x02\x03", t)                            # identical payload every 100 ms
            results.append((round(t, 1), w.update(True, 0.5, t)))
            t += 0.1
        self.assertEqual([r for r in results if r[1] != OK], [(0.5, FLATLINE)])
        self.assertTrue(w.forcing_zero)

    def test_flatline_clock_resets_on_change(self):
        w = SensorWatchdog()
        t = 0.0
        for payload in [b"a"] * 4 + [b"b"] + [b"b"] * 4 + [b"c"]:   # never 0.5 s identical
            w.feed(payload, t)
            self.assertEqual(w.update(True, 0.5, t), OK)
            t += 0.1

    def test_disarm_resets_latch(self):
        w = SensorWatchdog()
        w.update(True, 0.5, 0.0); w.update(True, 0.5, 0.6)
        self.assertTrue(w.forcing_zero)
        w.update(False, 0.0, 0.7)                                   # disarmed -> reset
        self.assertFalse(w.forcing_zero)
        self.assertEqual(w.update(True, 0.5, 0.8), OK)              # re-engages cleanly


class TestSenderPingAndWatchdog(unittest.TestCase):
    def setUp(self):
        self.master, self.slave, self.path = make_pty()
        self.link = SerialLink(self.path); self.link.open()

    def tearDown(self):
        self.link.close(); os.close(self.slave); os.close(self.master)

    def test_pings_every_250ms_until_answered(self):
        s = CrsfSender(self.link, hz=100.0)
        s.start(); s.set_controls(Controls())
        t0 = time.monotonic()
        data = b""
        while time.monotonic() - t0 < 1.1:
            s.set_controls(Controls()); data += read_available(self.master, 0.05)
        s.mark_device_info(crsf.ADDR_RX)                        # RX answered: stop pinging it
        mid = len(data)
        t1 = time.monotonic()
        while time.monotonic() - t1 < 0.8:
            s.set_controls(Controls()); data += read_available(self.master, 0.05)
        s.stop()
        frames = crsf.FrameParser().feed(data)
        pings = [f for f in frames if f.type == crsf.TYPE_DEVICE_PING]
        dest = lambda a, fs: sum(1 for f in fs if f.payload[0] == a)
        first = crsf.FrameParser().feed(data[:mid])
        first_pings = [f for f in first if f.type == crsf.TYPE_DEVICE_PING]
        print(f"\n  pings in first 1.1 s: TX={dest(0xEE, first_pings)} RX={dest(0xEC, first_pings)} FC={dest(0xC8, first_pings)}")
        for a in (0xEE, 0xEC, 0xC8):
            self.assertTrue(4 <= dest(a, first_pings) <= 6, dest(a, first_pings))   # ~every 250 ms
        late = [f for f in crsf.FrameParser().feed(data[mid:]) if f.type == crsf.TYPE_DEVICE_PING]
        self.assertEqual(dest(0xEC, late), 0)                      # RX no longer pinged
        self.assertGreater(dest(0xEE, late), 1)                    # TX still pinged

    def test_watchdog_forces_throttle_zero_when_sensor_silent(self):
        s = CrsfSender(self.link, hz=100.0, allow_arm=True, ping_targets=(), watchdog=SensorWatchdog())
        s.set_controls(Controls(throttle=0.5, arm=True))
        s.start()
        t0 = time.monotonic(); data = b""
        while time.monotonic() - t0 < 1.2:
            s.set_controls(Controls(throttle=0.5, arm=True))   # armed, 50% throttle, no sensor samples at all
            data += read_available(self.master, 0.05)
        s.stop()
        chans = [crsf.unpack_channels(f.payload) for f in crsf.FrameParser().feed(data) if f.type == crsf.TYPE_RC_CHANNELS]
        thr = [c[2] for c in chans[:-1]]                         # last frame is the safe shutdown frame
        self.assertIn(throttle01_to_crsf(0.5), thr)               # throttle was passed through at first
        trip = next(i for i, v in enumerate(thr) if v == 172)
        print(f"\n  throttle forced to 172 after {trip} frames (~{trip * 0.01:.2f} s at 100 Hz)")
        self.assertTrue(40 <= trip <= 70)                         # about 0.5 s
        self.assertTrue(all(v == 172 for v in thr[trip:]))        # stays latched
        self.assertTrue(all(c[4] == 1811 for c in chans[:-1]))    # arm channel itself untouched (as in Nimbus)

    def test_watchdog_stays_quiet_with_live_sensor(self):
        s = CrsfSender(self.link, hz=100.0, allow_arm=True, ping_targets=(), watchdog=SensorWatchdog())
        s.set_controls(Controls(throttle=0.5, arm=True))
        s.feed_sensor(b"\x00\x07")
        s.start()
        t0 = time.monotonic(); data = b""; i = 0
        while time.monotonic() - t0 < 1.5:
            s.set_controls(Controls(throttle=0.5, arm=True))
            s.feed_sensor(bytes([i % 256, 7])); i += 1            # changing payload at ~20 Hz
            data += read_available(self.master, 0.05)
        s.stop()
        thr = [crsf.unpack_channels(f.payload)[2] for f in crsf.FrameParser().feed(data) if f.type == crsf.TYPE_RC_CHANNELS][:-1]
        self.assertTrue(all(v == throttle01_to_crsf(0.5) for v in thr))


class TestOptrange(unittest.TestCase):
    def test_decode_optrange_with_fc_address_prefix(self):
        body = struct.pack(">HBBhhBB", 490, 100, 1, 3, -2, 90, 1)       # 490 mm = 0x01EA: its low byte equals the radio address
        plain = crsf.decode_optrange(body)
        self.assertEqual(crsf.decode_optrange(b"\x00\xc8" + body), plain)
        self.assertAlmostEqual(plain["distance_m"], 0.49)

    def test_decode_optrange_layout_and_scaling(self):
        # distance 1500 mm, strength 255, precision 3, flow_x +200, flow_y -50, quality 64, status 1
        payload = struct.pack(">HBBhhBB", 1500, 255, 3, 200, -50, 64, 1)
        d = crsf.decode_telemetry(crsf.Frame(0xC8, crsf.TYPE_OPTRANGE, payload))
        self.assertEqual(d[0], "optrange")
        v = d[1]
        self.assertAlmostEqual(v["distance_m"], 1.5)
        self.assertEqual((v["dist_strength"], v["dist_precision"], v["flow_quality"], v["flow_status"]), (255, 3, 64, 1))
        self.assertAlmostEqual(v["velocity_forward"], 1.5 * 2.0)     # distance * flow_x / 100
        self.assertAlmostEqual(v["velocity_right"], -1.5 * 0.5)      # +distance * flow_y / 100 (2026-11-23 owner bench: raw flow_y + = right)

    def test_distance_validity_window(self):
        for raw, ok in [(0, False), (1, True), (5999, True), (6000, False), (0xFFFF, False)]:
            v = crsf.decode_optrange(struct.pack(">HBBhhBB", raw, 0, 0, 0, 0, 0, 0))
            self.assertEqual(v["distance_m"] != 0.0, ok, raw)

    def test_short_payload_rejected(self):
        self.assertIsNone(crsf.decode_optrange(bytes(9)))

class TestPortLock(unittest.TestCase):
    def test_second_open_of_the_same_port_is_refused(self):
        import errno as _errno
        from nimbus_pc.emulator import DongleEmulator
        from nimbus_pc.sender import SerialLink
        emu = DongleEmulator().start()
        a, b = SerialLink(emu.path), SerialLink(emu.path)
        try:
            a.open()
            with self.assertRaises(OSError) as cm:
                b.open()
            self.assertEqual(cm.exception.errno, _errno.EBUSY)
            self.assertIn("another DroneForge program", str(cm.exception))
            a.close()
            b2 = SerialLink(emu.path); b2.open(); b2.close()          # free again once the first program lets go
        finally:
            a.close(); emu.stop()


if __name__ == "__main__":
    unittest.main(verbosity=2)


class TestPingRearm(unittest.TestCase):
    def test_pings_stop_at_the_cap_and_restart_when_the_link_comes_up(self):
        from nimbus_pc.sender import CrsfSender
        s = CrsfSender(link=None, ping_targets=(0xEE, 0xEC, 0xC8))
        s._ping_due.clear()                                  # as if the cap had been reached for all
        s._answered.add(0xEE); s.rearm_pings()
        self.assertEqual(sorted(s._ping_due), [0xC8, 0xEC])  # TX already answered: only RX and FC are pinged again
        self.assertEqual(s._ping_count[0xEC], 0)


class TestKinematicDecode(unittest.TestCase):
    def test_decode_with_and_without_prefix_and_invalid_markers(self):
        body = struct.pack(">9h", 120, -80, 503, 30, -20, 5, 100, -200, 15700)
        d = crsf.decode_kinematic(b"\x00\xc8" + body)
        self.assertEqual(d, crsf.decode_kinematic(body)); self.assertAlmostEqual(d["z_m"], 0.503); self.assertAlmostEqual(d["vx_mps"], 0.03); self.assertTrue(d["valid"])
        self.assertFalse(crsf.decode_kinematic(struct.pack(">9h", 0x7FFF, -32768, 0, 0, 0, 0, 0, 0, 0))["valid"])
        self.assertIsNone(crsf.decode_kinematic(bytes(10)))
