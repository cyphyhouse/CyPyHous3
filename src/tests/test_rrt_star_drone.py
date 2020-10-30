import random
from collections import Sequence
from unittest import TestCase

import numpy as np

from src.motion.cylobs import CylObs
from src.motion.pos_types import Pos, pos3d, Seg
from src.motion.rectobs import RectObs
from src.motion.rrt_star_drone import RRT as RRTStarDrone


class TestRRTStarDrone(TestCase):
    def setUp(self) -> None:
        random.seed(0)
        self.planner = RRTStarDrone(
            rand_area=(-10, 10, -10, 10, 0.5, 5.5),
            expand_dis=2.0,
            max_iter=500
        )
        self.obstacles = [
            CylObs(Pos(np.array([3.8239, 3.403029, 0])), radius=0.5, height=1.6),
            RectObs(Pos(np.array([1.0, 3.4, 3.0])), np.array([5.0, 2.0, 6.0])),
            RectObs(Pos(np.array([-4.13394, 3.4, 3.0])), np.array([5.0, 2.0, 6.0])),
            RectObs(Pos(np.array([1.0, 4.7, 3.0])), np.array([5.0, 2.0, 6.0])),
            RectObs(Pos(np.array([-4.13394, 4.7, 3.0])), np.array([5.0, 2.0, 6.0])),
            RectObs(Pos(np.array([1.0, -1.5, 3.0])), np.array([5.0, 2.0, 6.0])),
            RectObs(Pos(np.array([-4.13394, -1.5, 3.0])), np.array([5.0, 2.0, 6.0])),
            RectObs(Pos(np.array([1.0, -2.8, 3.0])), np.array([5.0, 2.0, 6.0])),
            RectObs(Pos(np.array([-4.13394, -2.8, 3.0])), np.array([5.0, 2.0, 6.0])),
            RectObs(Pos(np.array([1.0, -7.8, 3.0])), np.array([5.0, 2.0, 6.0])),
            RectObs(Pos(np.array([-4.13394, -7.8, 3.0])), np.array([5.0, 2.0, 6.0])),
            RectObs(Pos(np.array([1.0, -9.1, 3.0])), np.array([5.0, 2.0, 6.0])),
            RectObs(Pos(np.array([-4.13394, -9.1, 3.0])), np.array([5.0, 2.0, 6.0])),
            RectObs(Pos(np.array([-8.34545, 0.0, 1.4])), np.array([0.2, 7.5, 2.8])),
            RectObs(Pos(np.array([-3.5, 9.0, 1.4])), np.array([7.5, 0.2, 2.8]))
        ]

    def check_static_obstacle(self, start: Pos, path) -> bool:
        prev_pos = start
        ret = True
        for pos in path:
            ret = ret and all(obs.isdisjoint(Seg(prev_pos, pos)) for obs in self.obstacles)
            prev_pos = pos
        return ret

    def test_find_path_1(self):
        start = pos3d(-3.0, 0.2, 2.0)
        path = self.planner.find_path(
            start=start,
            end=pos3d(2.5, 6.3, 3.0),
            obstacle_list=self.obstacles
        )
        assert path
        self.assertTrue(self.check_static_obstacle(start, path),
                        "Returned path intersects static obstacles")

    def test_find_path_2(self):
        start = pos3d(8.00016567611682,1.9999447874211873,0.7324747854557848)
        path = self.planner.find_path(
            start=start,
            end=pos3d(+3.0, 1.8, 4.0),
            obstacle_list=self.obstacles
        )
        assert path
        self.assertTrue(self.check_static_obstacle(start, path),
                        "Returned path intersects static obstacles")
