import unittest
import numpy as np
from nimbus_core.rectify import load_o4_rectifier


class TestO4Rectifier(unittest.TestCase):
    def test_1080p_becomes_the_640x480_pinhole_the_vo_needs(self):
        out = load_o4_rectifier(1920, 1080)(np.zeros((1080, 1920, 3), np.uint8))
        self.assertEqual(out.shape, (480, 640, 3))


if __name__ == "__main__":
    unittest.main()
