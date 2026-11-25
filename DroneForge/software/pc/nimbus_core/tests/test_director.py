import json
import os
import tempfile
import time
import unittest

from nimbus_core import settings as S
from nimbus_core.core import NimbusCore
from nimbus_core.director import FlightDirector, FlightState, evaluate_preflight

GOOD = dict(connected=True, simulated=False, link_lq=100, telemetry_age_s=0.1, fc_armed=False, rest_tilt_deg=0.1, range_m=0.07, range_age_s=0.1,
            autonomy_idle=True, safety_latch=None, battery_v=16.0, fc_usb=False, height_m=0.5, ceiling_m=3.0)


def verdict(**kw):
    cs = evaluate_preflight(dict(GOOD, **kw))
    return {c.id: c for c in cs}


class TestPreflight(unittest.TestCase):
    def test_all_good(self):
        self.assertTrue(all(c.ok for c in evaluate_preflight(GOOD)))

    def test_table(self):
        # (override, check id, ok, blocking): only connection, armed state, a running flight and a safety latch block
        rows = [
            (dict(connected=False), "connected", False, True),
            (dict(link_lq=79), "link", False, False), (dict(link_lq=80), "link", True, False), (dict(link_lq=None), "link", False, False),
            (dict(telemetry_age_s=1.0), "telemetry", False, False), (dict(telemetry_age_s=None), "telemetry", False, False),
            (dict(fc_armed=True), "disarmed", False, True),
            (dict(rest_tilt_deg=0.49), "rest_tilt", True, False), (dict(rest_tilt_deg=1.0), "rest_tilt", False, False), (dict(rest_tilt_deg=1.6), "rest_tilt", False, False),
            (dict(range_m=0.01), "range", False, False), (dict(range_m=0.15), "range", True, False), (dict(range_m=0.16), "range", False, True), (dict(range_m=0.2), "range", False, True), (dict(range_m=None), "range", False, False), (dict(range_age_s=2.0), "range", False, False),
            (dict(autonomy_idle=False), "autonomy_idle", False, True),
            (dict(safety_latch="inverted"), "safety", False, True),
            (dict(battery_v=None), "battery", False, False),
            (dict(fc_usb=True), "fc_usb", False, False),
            (dict(height_m=3.5), "ceiling", False, False), (dict(height_m=3.0), "ceiling", True, False), (dict(height_m=1.0, ceiling_m=0.8), "ceiling", False, False),
        ]
        for over, cid, ok, blocking in rows:
            c = verdict(**over)[cid]
            self.assertEqual((c.ok, c.blocking), (ok, blocking), (over, cid, c))

    def test_simulated_skips_hardware_checks(self):
        v = verdict(simulated=True, link_lq=None, telemetry_age_s=None, rest_tilt_deg=None, range_m=None, range_age_s=None, fc_usb=True)
        self.assertTrue(all(c.ok for k, c in v.items() if k not in ("battery",) or True))


class TestDirectorSim(unittest.TestCase):
    def setUp(self):
        self.core = NimbusCore(); self.core.connect_sim(); self.d = self.core.director
        for _ in range(250): self.core.tick(); time.sleep(0.02)   # telemetry must flow before the range/rate checks pass

    def tearDown(self):
        self.core.disconnect()

    def test_disconnected_refusals_and_state(self):
        c = NimbusCore(); d = c.director
        self.assertEqual(d.state().phase, "disconnected")
        for r in (d.arm(), d.takeoff(), d.land(), d.disarm()):
            self.assertFalse(r.ok); self.assertIsInstance(r.message, str)
        self.assertTrue(d.safe().ok)

    def test_arm_grants_internally_and_disarm_revokes(self):
        d, core = self.d, self.core
        self.assertFalse(core.allow_arm)
        r = d.arm(); self.assertTrue(r.ok, r.message)
        self.assertTrue(core.allow_arm); self.assertEqual(core.source, "external")
        self.assertEqual(d.state().phase, "arming")
        self.assertFalse(d.arm().ok)                                   # already arming
        for _ in range(300):
            core.tick(); time.sleep(0.01)
            if d.state().phase == "armed": break
        self.assertEqual(d.state().phase, "armed"); self.assertTrue(d.state().armed)
        self.assertTrue(d.disarm().ok); core.tick()
        self.assertFalse(core.allow_arm); self.assertEqual(d.state().phase, "ready")

    def test_takeoff_height_ceiling_land_and_safe(self):
        d, core = self.d, self.core
        r = d.takeoff(2.0); self.assertTrue(r.ok, r.message)
        self.assertAlmostEqual(d.state().target_m, 2.0)
        self.assertFalse(d.takeoff().ok)                               # already running
        self.assertFalse(d.set_height(3.5).ok)                         # above the ceiling
        self.assertFalse(d.set_height(0.1).ok)                         # below limit
        self.assertTrue(d.safe().ok)
        self.assertFalse(core.allow_arm); self.assertEqual(core.source, "idle"); self.assertEqual(d.state().phase, "ready")
        self.assertFalse(d.takeoff(3.5).ok)                            # ceiling refusal returned as a message
        self.assertEqual(core.autonomy.phase, "idle")

    def test_state_fields(self):
        st = self.d.state()
        self.assertIsInstance(st, FlightState)
        with self.assertRaises(Exception): st.phase = "x"              # frozen

    def test_armed_takeoff_then_land(self):
        d, core = self.d, self.core
        core.autonomy.set_settings(S.FlightSettings(0.5, 0.42, 0.7), persist=False)
        self.assertTrue(d.arm().ok)
        for _ in range(300):
            core.tick(); time.sleep(0.01)
            if d.state().phase == "armed": break
        self.assertTrue(d.takeoff().ok)
        self.assertIn(d.state().phase, ("takeoff", "climbing", "holding"))
        self.assertTrue(d.land().ok); self.assertEqual(d.state().phase, "landing")


class TestCeilingViaDirector(unittest.TestCase):
    def test_ceiling_drives_safety(self):
        core = NimbusCore(); au = core.autonomy
        self.assertEqual(au.set_settings(S.FlightSettings(0.5, 0.42, 0.7, ceiling_m=1.5), persist=False), [])
        self.assertEqual(core.safety.cfg.max_height_m, 1.5)
        self.assertTrue(au.set_settings(S.FlightSettings(2.0, 0.42, 0.7, ceiling_m=1.5), persist=False))   # hover above ceiling refused


class TestSettingsMigration(unittest.TestCase):
    def _load(self, d):
        p = os.path.join(tempfile.mkdtemp(), "s.json"); json.dump(d, open(p, "w")); return S.load(p)

    def test_unknown_keys_ignored(self):
        self.assertEqual(self._load({"hover_height": 0.5, "flight_controller": "fc", "fc_hold": True}).hover_height, 0.5)
        self.assertEqual(self._load({"hover_height": 0.5}).hover_height, 0.5)
        self.assertEqual(S.FlightSettings().ceiling_m, 3.0)
        p = os.path.join(tempfile.mkdtemp(), "s.json"); S.save(S.FlightSettings(0.4, 0.4, 0.6, ceiling_m=1.0), p)
        self.assertEqual(S.load(p).ceiling_m, 1.0); self.assertNotIn("flight_controller", json.load(open(p)))

    def test_ceiling_validation(self):
        for c in (0.2, 3.1): self.assertTrue(S.FlightSettings(0.2, ceiling_m=c).validate())
        self.assertTrue(S.FlightSettings(1.0, ceiling_m=0.8).validate())


class TestSchema(unittest.TestCase):
    def test_complete(self):
        keys = set(S.FIELD_KEYS)
        for must in ("hover_height", "ceiling_m", "thr_hover0", "thr_max", "creep_per_s", "batt_ref_v", "xy_source"):
            self.assertIn(must, keys)
        defaults = S.FlightSettings(); names = {g["group"] for g in S.SCHEMA}
        self.assertEqual(names, {"Flight", "Safety", "Controller", "Battery", "Height loop", "Estimation"})
        for g in S.SCHEMA:
            for f in g["fields"]:
                for k in ("key", "label", "unit", "kind", "min", "max", "step", "choices", "tooltip", "experimental", "experimental_values", "readonly"):
                    self.assertIn(k, f, f)
                self.assertTrue(f["label"] and f["tooltip"])
                self.assertIn(f["kind"], ("number", "choice"))
                if f["readonly"]: continue
                self.assertTrue(hasattr(defaults, f["key"]), f["key"])
                if f["kind"] == "number":
                    self.assertLess(f["min"], f["max"]); self.assertGreater(f["step"], 0)
                    self.assertTrue(f["min"] <= getattr(defaults, f["key"]) <= f["max"], f["key"])
                else:
                    self.assertIn(getattr(defaults, f["key"]), [v for v, _ in f["choices"]])


if __name__ == "__main__":
    unittest.main()


class TestRefusedTakeoffKeepsArmed(unittest.TestCase):
    def test_a_refused_takeoff_from_armed_does_not_disarm(self):
        from nimbus_core.director import FlightDirector
        class AU:                                                    # minimal stand-in for the autonomy layer
            phase = "armed"; in_flight = False; message = "takeoff refused: range data too slow"; gyro_cal_before_arm = False
            settings = type("S", (), {"hover_height": 0.5})()
            def takeoff(self): return False
        d = FlightDirector.__new__(FlightDirector); d.core = type("C", (), {"autonomy": AU(), "notify": lambda self, *a, **k: None, "state_source": "live"})(); calls = []
        import time; d.clock = time.monotonic; d._cmds = {}; d._cmd_seq = 0; d._safed = False
        d._refusal = lambda skip=(): ""; d._grant = lambda: calls.append("grant"); d._revoke = lambda: calls.append("revoke")
        r = d.takeoff(); self.assertFalse(r.ok); self.assertNotIn("revoke", calls)           # still armed and permitted
        AU.phase = "idle"; calls.clear(); r = d.takeoff(); self.assertFalse(r.ok); self.assertIn("revoke", calls)    # from idle the permission is dropped again


class TestStartSequence(unittest.TestCase):
    """Real-drone start: gyro calibration -> 3-2-1 countdown -> estimator reset -> arm (the sim/bench skip the waiting)."""

    def _au(self):
        from nimbus_core.autonomy import HoverAutonomy
        core = type("C", (), {})(); core.state_source = "live"; core.connected = True; core.allow_arm = True; core.source = "external"
        core.safety = type("S", (), {"disarm_latched": None, "cfg": type("K", (), {})()})(); core.event = lambda *a, **k: None
        core.bus = type("B", (), {"subscribe": lambda self, *a, **k: None})()
        au = HoverAutonomy(core); au.phase = "arming"; au._t_arm = 100.0; au._arm_only = True; au._est_reset = False; au.gyro_cal_before_arm = True
        return au

    def test_stages_in_order_and_no_arm_before_the_countdown_ends(self):
        au = self._au()
        c = au.controls(101.0); self.assertFalse(c.arm); self.assertIn("Gyro calibration", au.message)            # 1) gyro cal (gesture: pitch/yaw sticks)
        c = au.controls(104.5); self.assertFalse(c.arm); self.assertEqual(au.message, "Starting in 3...")           # 2) countdown 3, 2, 1
        au.controls(105.5); self.assertEqual(au.message, "Starting in 2...")
        c = au.controls(106.5); self.assertFalse(c.arm); self.assertEqual(au.message, "Starting in 1...")
        old = au.ctl; c = au.controls(107.5); self.assertTrue(c.arm)                                               # 3) estimator reset, then 4) arm
        self.assertIsNot(au.ctl, old); self.assertTrue(au._est_reset)

    def test_simulator_skips_the_waiting(self):
        au = self._au(); au.core.state_source = "sim"
        c = au.controls(100.0); self.assertTrue(c.arm); self.assertFalse(au._est_reset)
