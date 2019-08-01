"""
demo path planner
"""

import numpy as np

from src.motion.planner import Planner
from src.motion.pos_types import Pos


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
        mid_point = Pos(np.array([(start_point.x + end_point.x) / 2, (start_point.y + end_point.y) / 2, (
                start_point.z + end_point.z) / 2]))
        path = [start_point, mid_point, end_point]
        return path


'''
a = DemoPlan()
c = a.find_path(Pos(np.array([0, 0, 0])), Pos(np.array([1, 1, 0])))
print(c)
'''
