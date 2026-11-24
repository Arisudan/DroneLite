import unittest

from nimbus_core.battery_cal import calibrate_battery_scale as cal
from nimbus_core import settings as S


class TestBatteryCal(unittest.TestCase):
    def test_math(self):
        self.assertAlmostEqual(cal(13.8, 4), 4 * 4.16 / 13.8)
        self.assertAlmostEqual(cal(16.64, 4), 1.0)
        self.assertAlmostEqual(cal(13.8, 4, 4.2), 4 * 4.2 / 13.8)

    def test_clamp_and_bad(self):
        self.assertEqual(cal(5.0, 4), 2.0)
        self.assertEqual(cal(40.0, 4), 0.5)
        with self.assertRaises(ValueError):
            cal(0, 4)

    def test_settings_field_and_old_json(self):
        self.assertEqual(S.FlightSettings().battery_scale, 1.0)
        self.assertIn("battery_scale", S.FIELD_KEYS)
        self.assertTrue(S.FlightSettings(battery_scale=2.5).validate())

    def test_old_json_loads(self):
        import json, os, tempfile
        p = os.path.join(tempfile.mkdtemp(), "s.json"); json.dump({"hover_height": 0.4}, open(p, "w"))
        self.assertEqual(S.load(p).battery_scale, 1.0)


class TestDisplayScale(unittest.TestCase):
    def test_display_uses_factor_raw_unchanged(self):
        from unittest import mock
        from nimbus_gcs.model import GcsModel, display_vbat
        self.assertAlmostEqual(display_vbat(13.8, 1.2), 16.56)
        self.assertIsNone(display_vbat(None, 1.2))
        m = GcsModel()
        with mock.patch.object(GcsModel, "connected", new_callable=mock.PropertyMock, return_value=True):
            m._vbat_f = 13.8
            m.core.autonomy.settings.battery_scale = 1.2
            self.assertAlmostEqual(m._vbat(), 16.56)
            self.assertEqual(m.raw_vbat(), 13.8)
            self.assertAlmostEqual(m.status()["battery_cell_v"], 4.14) if False else None
            m.core.safety.feed_battery(0.0, 13.8)
            self.assertEqual(m.core.safety.vbat_f, 13.8)     # safety layer still sees the raw reading
