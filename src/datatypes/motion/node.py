#  Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

import numpy as np

from src.datatypes.motion.pos import Pos


class Node(Pos):
    """
    node object, primarily for optimized path planning.
    """

    def __init__(self, x, y, z=0.0, yaw=0.0):
        super(Node, self).__init__(np.array([x, y, z, yaw]))
        self.yaw = yaw
        self.path_x = []
        self.path_y = []
        self.path_yaw = []
        self.cost = 0.0
        self.parent = None

    def to_pos(self):
        return Pos(np.array([self.x, self.y, self.z, self.yaw]))


def to_node(p: Pos) -> Node:
    """
    converting a pos into a node
    :param p:
    :return:
    """
    return Node(p.x, p.y, p.z, p.yaw)
