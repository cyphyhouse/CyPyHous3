from typing import Union
import reeds_shepp
import numpy as np
from src.motion.planner import Planner
from src.motion.pos_types import Pos, Node, to_node

# TODO: Implement obstacle avoidance

class Reeds_Shepp_Planner(Planner):
    '''
    Class for Reeds Shepp Path Planning
    '''

    def __init__(self, rho: float = 0.75, step_size: float = 0.80):
        super(Reeds_Shepp_Planner, self).__init__()
        self.rho = rho
        self.step_size = step_size

    def find_path(self, start: Pos, end: Pos, obstacle_list: Union[list, None] = None) -> Union[list, None]:
        """
        Find a route from start to end with intermediate points
        :param start: starting vehicle position
        :param end: desired final vehicle position
        :param obstacle_list: list of obstacles (currently no functionality)
        :return:
        """
        if obstacle_list is None:
            obstacle_list = []
        start = to_node(start)
        end = to_node(end)
        if end.z != 0:
            print("z != 0, point not valid for car")
            return None
        computed_path = reeds_shepp.path_sample((start.x, start.y, start.yaw), (end.x, end.y, end.yaw), self.rho,
                                                self.step_size)
        final_path = []
        for tuple in computed_path:
            final_path.append(Node(tuple[0], tuple[1], 0))
        final_path.append(end)
        return final_path


'''
# Test case
start = Pos(np.array([0., 0., 0.]))
end = Pos(np.array([2.0, 1.5, 0.]))
rs = Reeds_Shepp_Planner().find_path(start, end)
print(rs)
'''
