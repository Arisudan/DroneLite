import os
import tempfile
import time
import unittest

from nimbus_gcs import log_store as L

OLD = time.time() - 60


def make(folder, stem, old=True):
    for s in L.SUFFIXES:
        p = os.path.join(folder, stem + s); open(p, "w").write("x" * 100)
        if old: os.utime(p, (OLD, OLD))


class LogStoreTests(unittest.TestCase):
    def setUp(self):
        self.t = tempfile.TemporaryDirectory(); self.d = self.t.name; make(self.d, "2026-11-11_180837_real-fc"); make(self.d, "2026-11-10_100000_simulator")

    def tearDown(self): self.t.cleanup()

    def test_list(self):
        r = L.list_recordings(self.d)
        self.assertEqual([e["stem"] for e in r], ["2026-11-11_180837_real-fc", "2026-11-10_100000_simulator"])
        self.assertEqual(r[0]["kind"], "real-fc"); self.assertEqual(len(r[0]["files"]), 3); self.assertEqual(r[0]["size"], 300)

    def test_delete_undo(self):
        dest, why = L.delete_recording(self.d, "2026-11-11_180837_real-fc"); self.assertEqual(why, "")
        self.assertEqual(len(os.listdir(dest)), 3); self.assertEqual(len(L.list_recordings(self.d)), 1)
        self.assertEqual(len(L.list_trash(self.d)), 1)
        self.assertEqual(L.restore(self.d, dest), (True, "")); self.assertEqual(len(L.list_recordings(self.d)), 2); self.assertFalse(os.path.exists(dest))

    def test_restore_conflict(self):
        dest, _ = L.delete_recording(self.d, "2026-11-11_180837_real-fc"); make(self.d, "2026-11-11_180837_real-fc")
        ok, why = L.restore(self.d, dest); self.assertFalse(ok); self.assertIn("already exists", why)

    def test_rename(self):
        ns, why = L.rename_recording(self.d, "2026-11-11_180837_real-fc", "hover test!#1")
        self.assertEqual(ns, "2026-11-11_180837_real-fc_hover_test__1")
        for s in L.SUFFIXES: self.assertTrue(os.path.exists(os.path.join(self.d, ns + s)))
        self.assertEqual(len(L.list_recordings(self.d)), 2)

    def test_label_limit_and_refusals(self):
        ns, _ = L.rename_recording(self.d, "2026-11-11_180837_real-fc", "a" * 90); self.assertEqual(len(ns.split("real-fc_")[1]), 40)
        self.assertIsNone(L.rename_recording(self.d, "2026-11-10_100000_simulator", "  ")[0])
        make(self.d, "2026-11-10_100000_simulator_dup")
        n, why = L.rename_recording(self.d, "2026-11-10_100000_simulator", "dup"); self.assertIsNone(n); self.assertIn("already exists", why)

    def test_live_refused(self):
        make(self.d, "2026-11-12_090000_real-fc", old=False)
        self.assertIsNone(L.delete_recording(self.d, "2026-11-12_090000_real-fc")[0])
        self.assertIsNone(L.rename_recording(self.d, "2026-11-12_090000_real-fc", "x")[0])
        live = os.path.join(self.d, "2026-11-10_100000_simulator.csv")
        d, why = L.delete_recording(self.d, "2026-11-10_100000_simulator", live); self.assertIsNone(d); self.assertIn("being written", why)
        self.assertEqual(len(L.list_recordings(self.d)), 3)

    def test_empty_trash_age(self):
        dest, _ = L.delete_recording(self.d, "2026-11-11_180837_real-fc")
        self.assertEqual(L.empty_trash_older_than(self.d, 30), 0)
        old = os.path.join(self.d, L.TRASH, "20250101T000000Z_2026-11-10_100000_simulator"); os.makedirs(old); open(os.path.join(old, "a.csv"), "w").close()
        self.assertEqual(L.empty_trash_older_than(self.d, 30), 1); self.assertEqual(len(L.list_trash(self.d)), 1)


if __name__ == "__main__":
    unittest.main()
