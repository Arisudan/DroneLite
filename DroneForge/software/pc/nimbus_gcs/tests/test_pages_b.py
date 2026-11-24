import os
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt6.QtWidgets import QApplication, QCheckBox, QMessageBox  # noqa: E402

from nimbus_gcs.main_window import MainWindow, apply_dark          # noqa: E402
from nimbus_gcs.components.sidebar_nav import SidebarNav, SectionStack  # noqa: E402
from nimbus_gcs.system_page import SetupPage                       # noqa: E402
from nimbus_gcs.settings_page import SettingsPage                  # noqa: E402

_app = QApplication.instance() or QApplication([])
apply_dark(_app)
SHOTS = os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "docs", "gcs_screens", "build_v5")


class TestSidebarNav(unittest.TestCase):
    def test_entries_state_text_and_search(self):
        n = SidebarNav()
        for k, t, s in (("a", "Sensors", "ok"), ("b", "Calibration", "caution"), ("c", "Radio", "setup")):
            n.add_entry(k, t, s)
        self.assertEqual([n.state_text(k) for k in "abc"], ["OK", "CAUTION", "NEEDS SETUP"])
        self.assertEqual(n.current(), "a")
        got = []; n.selected.connect(got.append); n.select("c"); self.assertEqual(got, ["c"])
        n.search.setText("cali"); self.assertEqual(n.visible_keys(), ["b"])
        n.search.setText("needs"); self.assertEqual(n.visible_keys(), ["c"])
        n.set_state("c", "ok"); self.assertEqual(n.state_text("c"), "OK")


class TestPagesB(unittest.TestCase):
    def setUp(self):
        for nm in ("question", "information", "warning", "critical"):
            setattr(QMessageBox, nm, staticmethod(lambda *a, **k: (_ for _ in ()).throw(AssertionError("dialog"))))

    def test_setup_page_sections_no_checkbox_and_arming_warning(self):
        w = MainWindow()
        try:
            p = SetupPage(w); p.refresh()
            self.assertEqual(p.hub.nav.keys(), ["sensors", "motors", "calibration", "radio"])      # Motor test restored 2026-11-23 (owner)
            warn = [l.text() for l in p.sections["motors"].findChildren(__import__("PyQt6.QtWidgets", fromlist=["QLabel"]).QLabel) if l.text().startswith("THIS ARMS THE DRONE")]
            self.assertTrue(warn)                                                                   # the arming warning is on the page
            self.assertEqual(p.findChildren(QCheckBox), [])
            p.hub.nav.select("calibration"); self.assertIs(p.hub.stack.currentWidget().widget(), p.sections["calibration"])
        finally:
            w.close()

    def test_settings_uses_sidebar_and_schema_groups(self):
        from nimbus_core import settings as S
        w = MainWindow()
        try:
            p = SettingsPage(w)
            self.assertEqual(p.nav.keys(), [g["group"] for g in S.SCHEMA])
            self.assertEqual(p.stack.count(), len(S.SCHEMA)); self.assertEqual(p.findChildren(QCheckBox), [])
        finally:
            w.close()

    def test_screenshots(self):
        os.makedirs(SHOTS, exist_ok=True)
        w = MainWindow(); w.resize(1920, 1080); w.show()
        try:
            for name, page in (("setup", SetupPage(w)), ("settings", SettingsPage(w))):
                page.resize(1920, 1080); page.show(); _app.processEvents()
                page.grab().save(os.path.join(SHOTS, name + ".png"))
        finally:
            w.close()


if __name__ == "__main__":
    unittest.main()
