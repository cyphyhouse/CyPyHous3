import unittest

import numpy as np

import src.datatypes.motion.pos as pos
import src.datatypes.motion.cylobs as obs
import src.datatypes.motion.seg as seg


class TestObs(unittest.TestCase):
    """
    testing the round obstacle type member access and collision check methods
    """

    def setUp(self):
        self.obs = obs.CylObs(pos.Pos(np.array([0, 0, 0])), 1.0)
        self.nc_path = seg.Seg(pos.Pos(np.array([2, 0, 0])), pos.Pos(np.array([2, 2, 0])))
        p1 = pos.Pos(np.array([-1.03, -0.1566, 1.10]))
        p2 = pos.Pos(np.array([1.094, 0.29, 1.22]))
        self.c_path = seg.Seg(p1, p2)

    def test_pos(self):
        self.assertEqual(self.obs.position, pos.Pos(np.array([0, 0, 0])), "testing position of obstacle")

    def test_size(self):
        self.assertEqual(self.obs.size, np.array([1]), "testing size of obstacle")

    def test_point_collision(self):
        self.assertTrue(self.obs._collision_point(pos.Pos(np.array([2, 0, 0]))), "testing point collision")
        self.assertFalse(self.obs._collision_point(pos.Pos(np.array([1, 0, 0]))), "testing point collision")

    def test_path_collision(self):
        self.assertTrue(self.obs._collision_path(self.nc_path), "testing path collision")
        self.assertFalse(self.obs._collision_path(self.c_path), "testing path collision")

    def test_collision(self):
        self.assertTrue(self.obs.collision_check(self.nc_path), "testing collision")
        self.assertFalse(self.obs.collision_check(pos.Pos(np.array([1, 0, 0]))), "testing collision")


if __name__ == '__main__':
    unittest.main()
