# Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

import unittest

import numpy as np

import src.datatypes.motion.pos as pos
import src.datatypes.motion.seg as seg


class TestSeg(unittest.TestCase):
    """
    testing the seg type member accesses and operations
    """

    def setUp(self):
        self.seg1 = seg.Seg(pos.Pos(np.array([1, 0, 0])), pos.Pos(np.array([1, 1, 0])))
        self.seg2 = seg.Seg(pos.Pos(np.array([1, 0, 1])), pos.Pos(np.array([1, 1, 1])))

    def test_start(self):
        self.assertEqual(self.seg1.start, pos.Pos(np.array([1, 0, 0])), "starting point access test")

    def test_end(self):
        self.assertEqual(self.seg1.end, pos.Pos(np.array([1, 1, 0])), "end point access test")

    def test_repr(self):
        self.assertEqual(str(self.seg1), str(self.seg1.start) + ":" + str(self.seg1.end), "string representation test")

    def test_len(self):
        self.assertEqual(self.seg1.length(), 1, "segment length test")
        self.assertEqual(self.seg2.length(), 1, "segment length test")

    def test_direction(self):
        self.assertEqual(self.seg1.direction(), pos.Pos(np.array([0, 1, 0])), "unit vector test")
        self.assertEqual(self.seg2.direction(), pos.Pos(np.array([0, 1, 0])), "unit vector test")


if __name__ == '__main__':
    unittest.main()
