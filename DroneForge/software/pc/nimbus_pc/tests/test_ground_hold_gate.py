"""Unit tests of the pure-C ground-hold gate (firmware/fc_matek_h743/source_snapshots/2026-11-10_ground-hold-gate_v5/ground_hold_gate.c) compiled with gcc and called via ctypes."""
import ctypes
import os
import subprocess
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.normpath(os.path.join(HERE, "..", "..", "..", "..", "firmware", "fc_matek_h743", "source_snapshots", "2026-11-10_ground-hold-gate_v5", "ground_hold_gate.c"))
DT = 0.02


class Gate(ctypes.Structure):
    _fields_ = [("inited", ctypes.c_bool), ("held", ctypes.c_bool), ("filt", ctypes.c_float), ("timer", ctypes.c_float)]


def load():
    out = os.path.join(tempfile.gettempdir(), "libghgate_test.so")
    subprocess.check_call(["gcc", "-O2", "-shared", "-fPIC", "-o", out, SRC, "-lm"])
    lib = ctypes.CDLL(out)
    lib.ghInit.argtypes = [ctypes.POINTER(Gate)]
    lib.ghUpdate.argtypes = [ctypes.POINTER(Gate), ctypes.c_float, ctypes.c_float, ctypes.c_bool]
    lib.ghUpdate.restype = ctypes.c_bool
    return lib


class GroundHoldGate(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.l = load()

    def setUp(self):
        self.g = Gate()
        self.l.ghInit(ctypes.byref(self.g))
        self.t = 0.0

    def step(self, h, armed=True):
        self.t += DT
        return self.l.ghUpdate(ctypes.byref(self.g), h, DT, armed)

    def run_for(self, h, secs):
        """returns list of (time, held) samples"""
        return [(self.t, self.step(h)) for _ in range(int(round(secs / DT)))]

    def test_floor_holds(self):
        self.assertTrue(all(h for _, h in self.run_for(0.01, 10.0)))
        self.assertTrue(all(h for _, h in self.run_for(0.045, 5.0)))

    def test_release_latency(self):
        self.run_for(0.01, 1.0)
        t0 = self.t
        s = self.run_for(0.12, 1.0)
        t_rel = next(t for t, h in s if not h)
        self.assertGreaterEqual(t_rel - t0, 0.15 - 1e-6)
        self.assertLessEqual(t_rel - t0, 0.45)
        self.assertTrue(all(not h for t, h in s if t >= t_rel))

    def test_flicker_releases_once_and_stays(self):
        self.run_for(0.01, 1.0)
        changes, prev = 0, True
        for i in range(int(20.0 / 0.25)):
            for _, h in self.run_for(0.07 if i % 2 else 0.01, 0.25):
                if h != prev:
                    changes += 1
                    prev = h
        self.assertEqual(changes, 1)
        self.assertFalse(prev)

    def test_reengage_only_after_1s_below_2cm(self):
        self.run_for(0.01, 0.5)
        self.run_for(0.20, 1.0)
        self.assertFalse(self.step(0.20))
        # 0.9 s at 1 cm (plus the filter settling): still released
        self.assertTrue(all(not h for _, h in self.run_for(0.01, 0.9)))
        # between 2 and 5 cm never re-engages
        self.assertTrue(all(not h for _, h in self.run_for(0.03, 5.0)))
        # a bump above 2 cm restarts the 1 s timer
        self.run_for(0.01, 0.8)
        self.run_for(0.03, 0.3)
        self.assertFalse(self.step(0.01))
        s = self.run_for(0.01, 2.0)
        t_re = next(t for t, h in s if h)
        self.assertGreaterEqual(t_re - (self.t - 2.0), 1.0 - 1e-6)
        self.assertLess(t_re - (self.t - 2.0), 1.5)
        self.assertTrue(all(h for t, h in s if t >= t_re))

    def test_disarm_resets(self):
        self.run_for(0.01, 0.5)
        self.run_for(0.20, 1.0)
        self.assertFalse(self.step(0.20))
        self.assertTrue(self.step(0.20, armed=False))
        self.assertTrue(self.step(0.01))


if __name__ == "__main__":
    unittest.main()
