from unittest import TestCase

import numpy as np

from src.motion.pos_types import Pos, Seg
from src.motion.rectobs import RectObs


class TestRectObs(TestCase):
    def setUp(self) -> None:
        self.obs = RectObs(Pos(np.array([0.5, 0.5, 0.0])), np.array([1.0, 1.0, 1.0]))

    def test_0_isdisjoint_point_true(self):
        p1 = Pos(np.array([1.5, 0.0, 1.0]))
        self.assertTrue(self.obs.isdisjoint(p1),
                        "Obstacle %s should be disjoint with point %s ." % (str(self.obs), str(p1)))

    def test_1_isdisjoint_point_false(self):
        p1 = Pos(np.array([0.8, 0.2, 0.3]))
        self.assertFalse(self.obs.isdisjoint(p1),
                         "Obstacle %s should intersect with point %s ." % (str(self.obs), str(p1)))

    def test_2_isdisjoint_seg_true(self):
        p1 = Pos(np.array([1.5, 1.0, 1.0]))
        p2 = Pos(np.array([1.1, 1.0, 1.0]))
        path = Seg(p1, p2)
        self.assertTrue(self.obs.isdisjoint(path),
                        "Obstacle %s should be disjoint with path %s ." % (str(self.obs), str(path)))

    def test_3_isdisjoint_seg_true(self):
        path = Seg(Pos(np.array([2.1, 2.1, 0.0])),
                   Pos(np.array([0.0, 2.1, 0.0])))
        self.assertTrue(self.obs.isdisjoint(path),
                        "Obstacle %s should be disjoint with path %s ." % (str(self.obs), str(path)))

    def test_3_isdisjoint_seg_false(self):
        path = Seg(Pos(np.array([1.5, 0.0, 0.0])),
                   Pos(np.array([0.0, 1.5, 0.0])))
        self.assertFalse(self.obs.isdisjoint(path),
                         "Obstacle %s should intersect with path %s ." % (str(self.obs), str(path)))

    def test_4_isdisjoint_seg_false(self):
        path = Seg(Pos(np.array([1.5, 0.0, 0.0])),
                   Pos(np.array([0.0, 0.5, 1.2])))
        self.assertFalse(self.obs.isdisjoint(path),
                         "Obstacle %s should intersect with path %s ." % (str(self.obs), str(path)))
