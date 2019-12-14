# Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

import unittest

import src.motion.planning.simplePlanner as planner


class TestPlanner(unittest.TestCase):
    """
    testing the obstacle abstract type member access methods
    TODO: add path testing
    """

    def setUp(self):
        self.planner = planner.SimplePlanner(2)

    def test_num_seg(self):
        self.assertEqual(self.planner.num_seg, 2, "test number of segments")


if __name__ == '__main__':
    unittest.main()
