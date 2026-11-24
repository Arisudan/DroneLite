import os
import select
import struct
import threading
import time
import unittest

from nimbus_pc import doctor
from nimbus_pc.emulator import DongleEmulator


def good_stats():
    return {"duration": 15.0, "hz": 100.0, "frames_sent": 1500, "max_interval_s": 0.011, "telemetry_frames": 3000, "crc_errors": 0,
            "device_info": {"name": "DIY C3 E28", "param_count": 20},
            "counts": {"link": 150, "battery": 10, "attitude": 10, "flight_mode": 10, "optrange": 300, "custom_d4": 50},
            "last": {"battery": {"voltage_v": 16.2}, "attitude": {"pitch_rad": 0.0, "roll_rad": 0.0}, "flight_mode": {"mode": "STAB*"},
                     "optrange": {"distance_m": 0.04, "dist_strength": 255, "flow_quality": 29}},
            "link": {"up_lq": [100] * 5, "up_snr_db": [13] * 5, "up_rssi1_dbm": [-48] * 5, "down_lq": [100] * 5, "rf_mode": [27],
                     "tx_power_idx": [1]},
            "msp_board": "board MATEKH743"}


def by_name(checks):
    return {c.name: c for c in checks}


class TestEvaluate(unittest.TestCase):
    def test_healthy_chain_has_no_fail(self):
        checks = doctor.evaluate(good_stats())
        self.assertEqual(doctor.verdict(checks), 0)
        self.assertEqual(by_name(checks)["radio link to the receiver"].status, doctor.PASS)
        self.assertEqual(by_name(checks)["range + flow frame (0xD3)"].status, doctor.PASS)

    def test_no_dongle_reply_fails(self):
        s = good_stats(); s["device_info"] = None
        c = by_name(doctor.evaluate(s))["dongle answers CRSF ping"]
        self.assertEqual(c.status, doctor.FAIL)
        self.assertIn("GPIO20/21", c.detail)

    def test_no_link_stats_fails_with_hint(self):
        s = good_stats(); s["link"] = {}; s["counts"].pop("link")
        c = by_name(doctor.evaluate(s))["radio link to the receiver"]
        self.assertEqual(c.status, doctor.FAIL)
        self.assertIn("hotspot", c.detail)

    def test_link_quality_thresholds(self):
        for lq, want in ((95, doctor.PASS), (80, doctor.WARN), (40, doctor.FAIL)):
            s = good_stats(); s["link"]["up_lq"] = [lq] * 5
            self.assertEqual(by_name(doctor.evaluate(s))["radio link to the receiver"].status, want, lq)

    def test_late_connection_is_judged_after_it_is_up(self):
        s = good_stats()
        s["link"]["t"] = [float(i) for i in range(10)]
        s["link"]["up_lq"] = [0, 0, 0, 0, 31, 95, 100, 100, 100, 100]
        for k in ("up_snr_db", "up_rssi1_dbm", "down_lq"):
            s["link"][k] = [13] * 10
        c = by_name(doctor.evaluate(s))["radio link to the receiver"]
        self.assertEqual(c.status, doctor.PASS, c.detail)
        self.assertIn("up after 5.0 s", c.detail)

    def test_never_connected_fails_and_dropouts_warn(self):
        s = good_stats(); s["link"]["up_lq"] = [0] * 5; s["link"]["t"] = [float(i) for i in range(5)]
        self.assertEqual(by_name(doctor.evaluate(s))["radio link to the receiver"].status, doctor.FAIL)
        s = good_stats()
        s["link"]["t"] = [float(i) for i in range(10)]
        s["link"]["up_lq"] = [100, 100, 100, 100, 0, 100, 100, 100, 100, 100]
        for k in ("up_snr_db", "up_rssi1_dbm", "down_lq"):
            s["link"][k] = [13] * 10
        c = by_name(doctor.evaluate(s))["radio link to the receiver"]
        self.assertEqual(c.status, doctor.WARN)
        self.assertIn("dropout", c.detail)

    def test_optrange_steadiness_check(self):
        name = "0xD3 steady enough for the 0.5 s sensor watchdog"
        s = good_stats(); s["optrange_t"] = [i * 0.1 for i in range(100)]
        self.assertEqual(by_name(doctor.evaluate(s))[name].status, doctor.PASS)
        s["optrange_t"] = [0.1 * i for i in range(50)] + [6.5 + 0.1 * i for i in range(50)]      # a 1.6 s hole
        c = by_name(doctor.evaluate(s))[name]
        self.assertEqual(c.status, doctor.FAIL)
        self.assertIn("Telem Ratio", c.detail)

    def test_slow_sender_and_jitter(self):
        s = good_stats(); s["frames_sent"] = 600
        self.assertEqual(by_name(doctor.evaluate(s))["RC frame rate and jitter"].status, doctor.FAIL)
        s = good_stats(); s["max_interval_s"] = 0.04
        self.assertEqual(by_name(doctor.evaluate(s))["RC frame rate and jitter"].status, doctor.WARN)

    def test_crc_errors(self):
        s = good_stats(); s["crc_errors"] = 1
        self.assertEqual(by_name(doctor.evaluate(s))["no CRC errors"].status, doctor.PASS)
        s["crc_errors"] = 12
        self.assertEqual(by_name(doctor.evaluate(s))["no CRC errors"].status, doctor.WARN)
        s["crc_errors"] = 600
        self.assertEqual(by_name(doctor.evaluate(s))["no CRC errors"].status, doctor.FAIL)

    def test_missing_optrange_and_flight_telemetry_warn(self):
        s = good_stats(); s["counts"].pop("optrange"); s["counts"].pop("battery")
        n = by_name(doctor.evaluate(s))
        self.assertEqual(n["range + flow frame (0xD3)"].status, doctor.WARN)
        self.assertIn("battery", n["flight telemetry (battery, attitude, mode)"].detail)

    def test_nimbus_only_frames_listed(self):
        n = by_name(doctor.evaluate(good_stats()))["Nimbus-specific frames"]
        self.assertEqual(n.status, doctor.INFO)
        self.assertIn("seen: custom_d4", n.detail)
        self.assertIn("fc_uid", n.detail)

    def test_fc_checks(self):
        s = good_stats()
        s["fc"] = {"reachable": True, "armed": False, "arming_mask": 0,
                   "spans": [(1244, 1756), (1244, 1756), (1244, 1756), (988, 988), (1000, 1000)]}
        n = by_name(doctor.evaluate(s))
        self.assertEqual(n["channels reach the FC"].status, doctor.PASS)
        self.assertEqual(n["FC disarmed"].status, doctor.PASS)
        s["fc"]["arming_mask"] = (1 << 3) | (1 << 25)
        self.assertIn("BADRX, ARMSWITCH", by_name(doctor.evaluate(s))["FC arming blockers (arm switch low)"].detail)
        s["fc"]["spans"] = [(1500, 1500)] * 3 + [(885, 885), (1000, 1000)]        # failsafe: nothing moves, throttle 885? still <=1100
        self.assertEqual(by_name(doctor.evaluate(s))["channels reach the FC"].status, doctor.FAIL)
        s["fc"]["spans"] = [(1244, 1756)] * 3 + [(988, 988), (2000, 2000)]       # arm channel high
        self.assertEqual(by_name(doctor.evaluate(s))["channels reach the FC"].status, doctor.FAIL)
        s["fc"]["armed"] = True
        self.assertEqual(by_name(doctor.evaluate(s))["FC disarmed"].status, doctor.FAIL)

    def test_report_text_and_json(self):
        checks = doctor.evaluate(good_stats())
        self.assertIn("-> OK", doctor.report(checks))
        self.assertTrue(doctor.report(checks, as_json=True).startswith("["))


class FakeFc:
    """Answers read-only MSP v1 requests 101/105/150 on a pty; RC values follow what the emulated dongle received."""

    def __init__(self, emu, armed=False, mask=0):
        self.master, self._slave = os.openpty()
        self.path = os.ttyname(self._slave)
        self.emu, self.armed, self.mask = emu, armed, mask
        self.seen = []
        self._stop = threading.Event()
        self._t = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._t.start(); return self

    def stop(self):
        self._stop.set(); self._t.join(1.0); os.close(self.master); os.close(self._slave)

    def _reply(self, cmd, payload):
        body = bytes([len(payload), cmd]) + payload
        c = 0
        for b in body:
            c ^= b
        os.write(self.master, b"$M>" + body + bytes([c]))

    def _run(self):
        buf = b""
        while not self._stop.is_set():
            if select.select([self.master], [], [], 0.05)[0]:
                buf += os.read(self.master, 64)
            while len(buf) >= 6 and buf.startswith(b"$M<"):
                cmd = buf[4]; buf = buf[6:]
                self.seen.append(cmd)
                if cmd == 101:
                    self._reply(101, struct.pack("<HHHIB", 123, 0, 0x33, 1 if self.armed else 0, 0) + bytes(1))
                elif cmd == 105:
                    ch = self.emu.last_channels or [992] * 16
                    ch = [ch[0], ch[1], ch[3], ch[2]] + list(ch[4:8])      # Betaflight MSP_RC order: roll, pitch, yaw, throttle, aux
                    us = [int(1000 + (v - 172) * 1000 / 1639) for v in ch[:8]]
                    self._reply(105, struct.pack("<8H", *us))
                elif cmd == 150:
                    p = struct.pack("<HHHI", 123, 0, 0x33, 0) + bytes([0]) + struct.pack("<HH", 20, 4) + bytes([0, 26]) \
                        + struct.pack("<I", self.mask) + bytes([0, 0, 0, 0, 0])
                    self._reply(150, p)
            if not buf.startswith(b"$M<"):
                buf = buf[-6:] if len(buf) > 6 else buf


class TestDoctorRun(unittest.TestCase):
    def setUp(self):
        self.emu = DongleEmulator().start()

    def tearDown(self):
        self.emu.stop()

    def test_run_against_emulator_dongle_only(self):
        stats = doctor.run(self.emu.path, duration=3.0, hz=100.0)
        n = by_name(doctor.evaluate(stats))
        for name in ("dongle answers CRSF ping", "radio link to the receiver", "flight telemetry (battery, attitude, mode)",
                     "range + flow frame (0xD3)", "MSP over CRSF (FC board info)", "no CRC errors"):
            self.assertEqual(n[name].status, doctor.PASS, (name, n[name].detail))
        self.assertGreater(self.emu.rc_frames, 100)
        arm_ch = [f for f in [self.emu.last_channels] if f][0][4]
        self.assertLess(arm_ch, 400)                       # never armed (CRSF 172 = 988 us)

    def test_run_with_fake_fc_checks_channels(self):
        fc = FakeFc(self.emu).start()
        try:
            stats = doctor.run(self.emu.path, fc_port=fc.path, duration=7.0, hz=100.0, msp=False)
        finally:
            fc.stop()
        n = by_name(doctor.evaluate(stats))
        self.assertEqual(n["flight controller reachable (USB MSP)"].status if "flight controller reachable (USB MSP)" in n
                         else doctor.PASS, doctor.PASS)
        self.assertEqual(n["channels reach the FC"].status, doctor.PASS, n["channels reach the FC"].detail)
        self.assertEqual(n["FC disarmed"].status, doctor.PASS)
        self.assertTrue(set(fc.seen) <= {101, 105, 150})            # read-only requests only

    def test_aborts_without_sending_when_fc_is_armed(self):
        fc = FakeFc(self.emu, armed=True).start()
        try:
            stats = doctor.run(self.emu.path, fc_port=fc.path, duration=2.0)
        finally:
            fc.stop()
        self.assertIn("armed", stats["aborted"])
        self.assertEqual(self.emu.rc_frames, 0)                       # nothing was sent


if __name__ == "__main__":
    unittest.main()
