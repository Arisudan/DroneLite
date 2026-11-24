"""Unit tests of the pure-C estimator gates (firmware/fc_matek_h743/source_snapshots/2026-11-09_estimator-gates_v4/estimator_gates.c) compiled with gcc and called via ctypes."""
import ctypes
import os
import subprocess
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.normpath(os.path.join(HERE, "..", "..", "..", "..", "firmware", "fc_matek_h743", "source_snapshots", "2026-11-09_estimator-gates_v4", "estimator_gates.c"))


class Gate(ctypes.Structure):
    _fields_ = [("rejected", ctypes.c_int)]


def load():
    out = os.path.join(tempfile.gettempdir(), "libestgates_test.so")
    subprocess.check_call(["gcc", "-O2", "-shared", "-fPIC", "-o", out, SRC, "-lm"])
    lib = ctypes.CDLL(out)
    f = ctypes.c_float
    lib.egRangeVarianceScale.argtypes = [f, ctypes.c_int, f]; lib.egRangeVarianceScale.restype = f
    lib.egFlowVarianceScale.argtypes = [f, f, f]; lib.egFlowVarianceScale.restype = f
    lib.egFlowYawScale.argtypes = [f]; lib.egFlowYawScale.restype = f
    lib.egSkipFlow.argtypes = [f]; lib.egSkipFlow.restype = ctypes.c_bool
    lib.egFlowGateInit.argtypes = [ctypes.POINTER(Gate)]
    lib.egFlowGate.argtypes = [ctypes.POINTER(Gate), f, f, f, f]; lib.egFlowGate.restype = ctypes.c_bool
    return lib


class EstimatorGates(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.l = load()

    def test_range_scale(self):
        r = self.l.egRangeVarianceScale
        self.assertAlmostEqual(r(1.5, 249, 0.35), 1.5, places=5)
        self.assertAlmostEqual(r(1.5, 250, 1.0), 9.0, places=5)
        self.assertAlmostEqual(r(1.5, 100, 0.349), 9.0, places=4)
        self.assertAlmostEqual(r(1.0, 255, 0.2), 6.0, places=5)

    def test_flow_rate_scale(self):
        f = self.l.egFlowVarianceScale
        self.assertAlmostEqual(f(1.0, 0, 0), 1.0, places=5)
        self.assertAlmostEqual(f(1.0, 20, 0), 1.0 + 2 * (20 * 0.0174532925) ** 2, places=4)
        self.assertAlmostEqual(f(1.0, 80, 0), 25.0, places=3)
        self.assertAlmostEqual(f(2.0, 0, 80), 50.0, places=2)
        self.assertGreater(f(1.0, 50, 0), f(1.0, 30, 0))
        self.assertAlmostEqual(f(1.0, 56.5685, 56.5685), 25.0, places=2)

    def test_yaw(self):
        y = self.l.egFlowYawScale
        self.assertEqual(y(0.74), 1.0)
        self.assertAlmostEqual(y(0.75), 1.0, places=5)
        self.assertAlmostEqual(y(-1.35), 51.0, places=3)
        self.assertGreater(y(1.0), 1.0)
        self.assertFalse(self.l.egSkipFlow(1.35))
        self.assertTrue(self.l.egSkipFlow(1.36))
        self.assertTrue(self.l.egSkipFlow(-1.5))

    def test_gate(self):
        g = Gate(); self.l.egFlowGateInit(ctypes.byref(g))
        G = lambda mx, my: self.l.egFlowGate(ctypes.byref(g), mx, my, 0.0, 0.0)
        self.assertTrue(G(0.9, -0.9))
        self.assertFalse(G(1.1, 0.0))
        self.assertFalse(G(0.0, -1.1))
        self.assertTrue(G(0.5, 0.5))          # good sample resets the count
        for _ in range(5):
            self.assertFalse(G(3.0, 0.0))
        self.assertTrue(G(3.0, 0.0))          # 6th in a row accepted (no lock-out)
        self.assertFalse(G(3.0, 0.0))         # counting restarts
        self.assertFalse(G(float("nan"), 0.0))


if __name__ == "__main__":
    unittest.main()
