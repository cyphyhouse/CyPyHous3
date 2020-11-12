# Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

import unittest

import numpy as np

import src.datatypes.motion.abstract.obstacle as obs
import src.datatypes.motion.pos as pos
import src.datatypes.motion.seg as seg


class DummyObs(obs.Obstacle):

    def __init__(self, point: pos.Pos, size: np.ndarray):
        super(DummyObs, self).__init__(point, size)

    def _collision_path(self, path: seg.Seg):
        return False

    def _collision_point(self, point: pos.Pos):
        return False


class TestObs(unittest.TestCase):
    """
    testing the obstacle abstract type member access methods
    """

    def setUp(self):
        self.obs = DummyObs(pos.Pos(np.array([1, 2, 3])), np.array([1]))
        self.pos = pos.Pos(np.array([4, 5, 6]))
        self.seg = seg.Seg(pos.Pos(np.array([1, 1, 1])), pos.Pos(np.array([2, 2, 3])))

    def test_pos(self):
        self.assertEqual(self.obs.position, pos.Pos(np.array([1, 2, 3])), "testing position of obstacle")

    def test_size(self):
        self.assertEqual(self.obs.size, np.array([1]), "testing size of obstacle")

    def test_path_collision(self):
        self.assertTrue(not (self.obs._collision_path(self.seg)), " testing dummy path collision")

    def test_pt_collision(self):
        self.assertTrue(not (self.obs._collision_point(self.pos)), " testing dummy point collision")


if __name__ == '__main__':
    unittest.main()
