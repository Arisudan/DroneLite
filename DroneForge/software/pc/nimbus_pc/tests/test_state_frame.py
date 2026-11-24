"""The compact state frame 0xD6 (proposed firmware change, simulation/decoder only): it must carry everything the controller takes from D3 + D4 losslessly,
in fewer bytes, and flow through the real parser and the core to the same messages."""
import math
import struct
import unittest

from nimbus_core.core import NimbusCore
from nimbus_pc import crsf as C


def d3_frame(dist, fx, fy, q, st):
    return C.encode_frame(C.TYPE_OPTRANGE, b"\x00\xc8" + struct.pack(">HBBhhBB", dist, 255, 1, fx, fy, q, st))


def d4_frame(x, y, z, vx, vy, vz, r, p, yw):
    return C.encode_frame(C.TYPE_MOTOR_PWM, b"\x00\xc8" + struct.pack(">9h", x, y, z, vx, vy, vz, r, p, yw))


def parse(raw):
    fr = C.FrameParser().feed(raw)
    assert len(fr) == 1
    return C.decode_telemetry(fr[0])


class TestStateFrame(unittest.TestCase):
    args = dict(distance_raw_mm=523, flow_x=-17, flow_y=33, flow_quality=88, ekf_valid=True, flow_status=1,
                vx_mm_s=-312, vy_mm_s=145, vz_mm_s=60, roll_e4=-120, pitch_e4=85, yaw_e4=15700)

    def test_carries_what_d3_and_d4_carried_and_is_smaller(self):
        a = self.args
        raw = C.encode_state(**a)
        name, d = parse(raw)
        self.assertEqual(name, "state_compact")
        got = dict(C_split for C_split in C.split_state(d))
        n3, o = parse(d3_frame(a["distance_raw_mm"], a["flow_x"], a["flow_y"], a["flow_quality"], a["flow_status"]))
        n4, k = parse(d4_frame(0, 0, 0, a["vx_mm_s"], a["vy_mm_s"], a["vz_mm_s"], a["roll_e4"], a["pitch_e4"], a["yaw_e4"]))
        for key in ("distance_m", "velocity_forward", "velocity_right", "flow_quality", "flow_status", "flow_x_raw", "flow_y_raw"):
            self.assertAlmostEqual(got["optrange"][key], o[key], places=9, msg=key)
        for key in ("vx_mps", "vy_mps", "vz_mps", "roll_rad", "pitch_rad", "yaw_rad", "valid"):
            self.assertEqual(got["kinematic"][key], k[key], key)
        self.assertTrue(all(math.isnan(got["kinematic"][k_]) for k_ in ("x_m", "y_m", "z_m")))      # position is not carried and is never invented
        self.assertEqual(len(raw), 26); self.assertEqual(len(d3_frame(1, 0, 0, 0, 0)) + len(d4_frame(*[0] * 9)), 40)

    def test_invalid_distance_and_flags(self):
        for raw_mm, dist in ((0, 0.0), (6000, 0.0), (1, 0.001), (5999, 5.999)):
            _, d = parse(C.encode_state(**dict(self.args, distance_raw_mm=raw_mm)))
            self.assertEqual(dict(C.split_state(d))["optrange"]["distance_m"], dist)
        _, d = parse(C.encode_state(**dict(self.args, ekf_valid=False, flow_status=0)))
        s = dict(C.split_state(d)); self.assertFalse(s["kinematic"]["valid"]); self.assertEqual(s["optrange"]["flow_status"], 0)

    def test_short_frame_is_rejected(self):
        self.assertIsNone(C.decode_state(b"\x00\xc8" + bytes(10)))

    def test_through_the_core_gives_range_velocity_and_attitude(self):
        core = NimbusCore(); seen = []
        core.bus.subscribe(__import__("nimbus_core.topics", fromlist=["T"]).TELEMETRY, lambda m: seen.append(m))
        _, d = parse(C.encode_state(**self.args))
        core._on_session_telemetry("state_compact", d)
        names = [m[0] for m in seen]
        self.assertIn("optrange", names); self.assertIn("kinematic", names); self.assertIn("attitude", names)     # attitude comes from the D4 half (attitude_select)
        att = next(m[1] for m in seen if m[0] == "attitude")
        self.assertAlmostEqual(att["roll_rad"], -0.012); self.assertAlmostEqual(att["pitch_rad"], 0.0085)

    def test_air_time_budget_at_the_target_rates(self):
        """Wire bytes per second: the current three frames at the measured 5.4 Hz and the compact frame at 10 and 15 Hz."""
        now = (16 + 24) * 5.4
        self.assertLess(now, 220)
        self.assertEqual(26 * 10, 260); self.assertEqual(26 * 15, 390)       # printed for the plan: the ceiling of the link decides which of these fits


if __name__ == "__main__":
    unittest.main()


class TestStateFrameAccounting(unittest.TestCase):
    def test_frame_statistics_count_it_as_d3_and_d4(self):
        from nimbus_core import topics as T
        core = NimbusCore(); got = []
        core.bus.subscribe(T.FRAME_IN, lambda f: got.append(f.type))
        core._publish_frame_in(C.Frame(C.ADDR_FC, C.TYPE_STATE, b""))
        core._publish_frame_in(C.Frame(C.ADDR_FC, C.TYPE_BATTERY, b""))
        self.assertEqual(got, [C.TYPE_OPTRANGE, C.TYPE_MOTOR_PWM, C.TYPE_BATTERY])


class TestHoldStatusBits(unittest.TestCase):
    a = dict(distance_raw_mm=500, flow_x=0, flow_y=0, flow_quality=90, ekf_valid=True, flow_status=1, vx_mm_s=0, vy_mm_s=0, vz_mm_s=0, roll_e4=0, pitch_e4=0, yaw_e4=0)

    def test_hold_bits_round_trip(self):
        for bits, act, flt, mode in ((0, False, False, 0), (0x04, True, False, 0), (0x04 | 0x10, True, False, 1), (0x04 | 0x20, True, False, 2), (0x08, False, True, 0)):
            _, d = parse(C.encode_state(**self.a, hold_bits=bits))
            k = dict(C.split_state(d))["kinematic"]
            self.assertEqual((k["hold_active"], k["hold_fault"], k["hold_mode"]), (act, flt, mode))
            self.assertTrue(k["valid"])                                      # the other flag bits are untouched
        _, d = parse(C.encode_state(**self.a, hold_bits=0xFF)); self.assertEqual(d["hold_mode"], 3)      # only bits 2-5 are taken from hold_bits
        self.assertTrue(dict(C.split_state(d))["optrange"]["flow_status"] == 1)

    def test_c_packer_matches_with_hold_bits(self):
        import os, subprocess, tempfile
        src = os.path.expanduser("~/droneforge/bf_fixed/repo/src/main/telemetry")
        if not os.path.exists(src + "/crsf_state_pack.c"): self.skipTest("firmware source not on this PC")
        t = tempfile.mkdtemp(); open(t + "/t.c", "w").write('#include <stdio.h>\n#include "crsf_state_pack.h"\nint main(void){uint8_t b[32];uint8_t n=crsfStatePack(b,523,-17,33,88,true,1,0x14,-312,145,60,-120,85,15700);for(int i=0;i<n;i++)printf("%02x",b[i]);return 0;}')
        subprocess.run(["gcc", "-I" + src, t + "/t.c", src + "/crsf_state_pack.c", "-o", t + "/t"], check=True)
        got = subprocess.run([t + "/t"], capture_output=True, text=True).stdout
        want = C.encode_state(distance_raw_mm=523, flow_x=-17, flow_y=33, flow_quality=88, ekf_valid=True, flow_status=1, vx_mm_s=-312, vy_mm_s=145, vz_mm_s=60, roll_e4=-120, pitch_e4=85, yaw_e4=15700, hold_bits=0x14, body_vel=True)[3:-1].hex()
        self.assertEqual(got, want)


class TestSidewaysStatusBit(unittest.TestCase):
    def test_xy_disabled_bit_round_trip(self):
        a = dict(distance_raw_mm=500, flow_x=0, flow_y=0, flow_quality=90, ekf_valid=True, flow_status=1, vx_mm_s=0, vy_mm_s=0, vz_mm_s=0, roll_e4=0, pitch_e4=0, yaw_e4=0)
        _, d = parse(C.encode_state(**a, hold_bits=0x80 | 0x04, body_vel=True)); k = dict(C.split_state(d))["kinematic"]
        self.assertTrue(k["xy_disabled"] and k["hold_active"] and k["vel_body"])
        _, d = parse(C.encode_state(**a, hold_bits=0x04, body_vel=True)); self.assertFalse(dict(C.split_state(d))["kinematic"]["xy_disabled"])
