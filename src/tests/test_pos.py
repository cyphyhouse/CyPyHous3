import unittest

import numpy as np
import numpy.testing

import src.datatypes.motion.pos as pos


class TestPos(unittest.TestCase):
    """
    testing the pos type member accesses and operations
    TODO: add tests for rospy related components. Maybe all in a separate file?
    """

    def setUp(self):
        self.point = pos.Pos(np.array([1, 2, 3, 4]))
        self.otherpt = pos.Pos(np.array([4, 5, 6, 7]))
        self.sum = self.point + self.otherpt
        self.diff = self.otherpt - self.point
        self.scalarprod = self.point * 3

    def test_x(self):
        self.assertEqual(self.point.x, 1, "x component access test")

    def test_y(self):
        self.assertEqual(self.point.y, 2, "y component access test")

    def test_z(self):
        self.assertEqual(self.point.z, 3, "z component access test")

    def test_yaw(self):
        self.assertEqual(self.point.yaw, 4, "yaw access test")

    def test_repr(self):
        self.assertEqual(str(self.point), "<1,2,3>", "string representation test")

    def test_eq(self):
        self.assertEqual(self.point, pos.Pos(np.array([1, 2, 3])), "equality test")

    def test_sum(self):
        self.assertEqual(self.sum, pos.Pos(np.array([5, 7, 9])), "sum test")

    def test_diff(self):
        self.assertEqual(self.diff, pos.Pos(np.array([3, 3, 3])), "difference test")

    def test_scalarprod(self):
        self.assertEqual(self.scalarprod, pos.Pos(np.array([3, 6, 9])), "scalar product test")
        self.assertEqual(self.scalarprod, 3 * self.point, "scalar product test")

    def test_mag(self):
        self.assertAlmostEqual(self.point.magnitude(), 3.74, 2, "magnitude test")

    def test_dir(self):
        xdir, ydir, zdir = self.point.direction().x, self.point.direction().y, self.point.direction().z
        self.assertAlmostEqual(xdir, 0.27, 2, "unit vector test")
        self.assertAlmostEqual(ydir, 0.53, 2, "unit vector test")
        self.assertAlmostEqual(zdir, 0.80, 2, "unit vector test")

    def test_toarr(self):
        numpy.testing.assert_allclose(self.point.to_arr(), np.array([1, 2, 3]))

    def test_tolist(self):
        self.assertEqual(self.point.to_list(), [1, 2, 3], "list conversion test")

    def test_distance(self):
        self.assertAlmostEqual(pos.distance(self.point, self.otherpt), 5.20, 2,
                               "test for distance between points")


if __name__ == '__main__':
    unittest.main()
