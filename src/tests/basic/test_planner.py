# Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

import unittest

import numpy as np

import src.motion.abstract.planner as planner
from src.datatypes.motion.pos import Pos


class DummyPlanner(planner.Planner):

    def __init__(self):
        super(DummyPlanner, self).__init__()

    def find_path(self, start_point: Pos, end_point: Pos, obstacles: list) -> list:
        return []


class TestPlanner(unittest.TestCase):
    """
    testing the obstacle abstract type member access methods
    """

    def setUp(self):
        self.planner = DummyPlanner()

    def test_path(self):
        self.assertEqual(self.planner.find_path(Pos(np.array([1, 2, 3])), Pos(np.array([2, 3, 4])), []), [],
                         "testing path of planner")


if __name__ == '__main__':
    unittest.main()
