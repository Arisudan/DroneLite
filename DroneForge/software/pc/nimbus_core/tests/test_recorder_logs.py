import os, tempfile, time, unittest
from nimbus_core import recorder as R


class TestLogRetention(unittest.TestCase):
    def setUp(self):
        self.d = tempfile.mkdtemp(); self.old = (R.DIR, R.MAX_FLIGHTS, R.MIN_FREE_M)
        R.DIR = self.d

    def tearDown(self):
        R.DIR, R.MAX_FLIGHTS, R.MIN_FREE_M = self.old

    def _mk(self, n):
        for i in range(n):
            for ext in (".csv", ".events.txt", ".summary.txt"):
                open(os.path.join(self.d, f"flight_20260101_0000{i:02d}{ext}"), "w").write("x")

    def test_oldest_deleted_first_when_over_count(self):
        self._mk(5); R.MAX_FLIGHTS = 3; R.MIN_FREE_M = 0
        gone = R.prune()
        self.assertEqual(gone, ["flight_20260101_000000", "flight_20260101_000001"])
        self.assertEqual(len(os.listdir(self.d)), 9)         # 3 flights x 3 files

    def test_deletes_to_make_room_when_disk_nearly_full(self):
        self._mk(4); R.MAX_FLIGHTS = 99; R.MIN_FREE_M = 1e12  # never satisfiable: everything old goes, nothing crashes
        self.assertEqual(len(R.prune()), 4)

    def test_summary_is_in_metres_and_reads_a_log(self):
        p = os.path.join(self.d, "flight_x.csv")
        with open(p, "w") as f:
            f.write(",".join(R.FIELDS) + "\n")
            for i in range(10):
                row = {k: "" for k in R.FIELDS}; row.update(t=i * 0.04, cmd_arm=1, range_m=0.5, batt_v=15.2, state="hover", z_est=0.5, z_ref=0.5, est_x=0.01 * i, est_y=0, fc_valid=1, fc_z=0.5, fc_x=0.01 * i, fc_y=0)
                f.write(",".join(str(row[k]) for k in R.FIELDS) + "\n")
        s = R.summarize(p)
        self.assertIn("0.500", s); self.assertIn("FC EKF", s); self.assertNotIn(" cm", s)


if __name__ == "__main__":
    unittest.main()
