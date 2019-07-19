"""
Path Planning Sample Code with RRT*

Author: AtsushiSakai(@Atsushi_twi)

Modifications for use in CyPhyHouse made by Amelia Gosse (gossea)
"""

import numpy as np

from src.motion.planner import Planner
from src.motion.pos import Pos


class DemoPlan(Planner):
    """
    Class for RRT* Planning
    """

    def __init__(self, randArea: list = None, expand_dis: float = 0.55555, goal_sample_rate: int = 200,
                 max_iter: int = 500):
        """
        initialization method for demo planner
        :param randArea:
        :param expand_dis:
        :param goal_sample_rate:
        :param max_iter:
        """
        if randArea is None:
            randArea = [-5, 5]
        super(DemoPlan, self).__init__()
        self.min_rand = randArea[0]
        self.max_rand = randArea[1]
        self.expand_dis = expand_dis
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter

    def find_path(self, start_point, end_point, obstaclelist=None) -> list:
        """
        demo Path Planning
        """
        if obstaclelist is None:
            obstaclelist = []
        mid_point = Pos(np.array[(start_point.x + end_point.x) / 2, (start_point.y + end_point.y) / 2, (
                start_point.z + end_point.z) / 2])
        path = [start_point, mid_point, end_point]
        return to_path(path)


def to_path(myList):
    returnpath = []
    for point in myList:
        returnpath.append(vec(point[0], point[1], 0))
    return returnpath


'''
a = RRT()
a.plan([0,0,0],[1,1,0])
c = a.Planning()
d = [vec(0.5,0.5,0)]
t = path_is_close(c,d,-1)
print(t)
'''
