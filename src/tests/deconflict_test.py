import unittest

import numpy as np

from src.CyPyHous3.src.motion.deconflict import is_collinear, line_to_pt_dist, seg_distance_3d
from src.CyPyHous3.src.motion.pos import Pos, Seg


class DeconflictTest(unittest.TestCase):
    """
    TODO: ADD UNIT TESTS FOR THE REMAINING FUNCTIONS
    """

    def setUp(self):
        pass

    def test_is_collinear(self):
        """
        test for collinear points
        :return:
        """
        p1 = Pos(np.array([0, 0, 0]))
        p2 = Pos(np.array([1, 0, 0]))
        p3 = Pos(np.array([1, 1, 1]))
        p4 = Pos(np.array([2, 0, 0]))
        t1 = is_collinear(p1, p2, p4)
        t2 = is_collinear(p1, p3, p4)
        self.assertEqual(t1, True, msg='collinear points not detected')
        self.assertEqual(t2, False, msg='non-collinear points not detected')

    def test_point_to_line_dist(self):
        """
        testing computation of point to line distance
        :return:
        """
        p1 = Pos(np.array([0, 0, 0]))
        p2 = Pos(np.array([2, 0, 0]))
        p4 = Pos(np.array([1, 0, 0]))
        p5 = Pos(np.array([0, 1, 0]))
        p6 = Pos(np.array([1, 1, 0]))
        s1 = Seg(p2, p4)
        s2 = Seg(p4, p5)
        dist1 = line_to_pt_dist(s1, p1)
        dist2 = line_to_pt_dist(s2, p6)
        self.assertEqual(dist1, 1., msg='line segment to point distance not '
                                        'computed correctly for a point along the line ')
        self.assertAlmostEqual(dist2, 0.707, 3, msg='line segment to point distance not '
                                                    'computed correctly for a point along the line ')

    def test_line_seg_3d_dist(self):
        """
        testing distance between two line segments in 3d
        :return:
        """
        p1 = Pos(np.array([0, 0, 0]))
        p4 = Pos(np.array([1, 0, 0]))
        p5 = Pos(np.array([0, 1, 0]))
        p6 = Pos(np.array([1, 1, 0]))
        s1 = Seg(p1, p5)
        s2 = Seg(p4, p6)
        self.assertEqual(seg_distance_3d(s1,s2), 1., msg='line segment distance in 3d not computed correctly')





if __name__ == '__main__':
    unittest.main()
