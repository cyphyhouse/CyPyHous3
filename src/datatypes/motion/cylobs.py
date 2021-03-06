# Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

import copy
import math

import numpy as np

from src.datatypes.motion.abstract.obstacle import Obstacle
from src.datatypes.motion.pos import Pos
from src.datatypes.motion.seg import Seg


class CylObs(Obstacle):
    """
    cylindrical obstacle, it is an infinitely long cylinder with a radius around
    """

    def __init__(self, point: Pos, radius: float):
        """
        :param point: position of obstacle
        :type point: Pos
        :param radius: radius of cylinder (obstacle)
        :type radius: float
        """
        point.z = 0
        super(CylObs, self).__init__(point, np.array([radius]))

    def _collision_point(self, point: Pos) -> bool:
        """
        checking whether a point "collides" with the obstacle
        :param point: position of point
        :return: boolean indicator of collision, true if no collision , false if collision
        """
        d = (self.position.x - point.x) ** 2 + (self.position.y - point.y) ** 2

        return math.sqrt(d) > self.size[0]

    def _collision_path(self, orig_path: Seg) -> bool:
        """
        checking if there is a collision with a path
        :param path: segment to check collision against
        :return: boolean indicator of collision, true if no collision, false if collision
        """
        path = copy.deepcopy(orig_path)
        path.start.z = 0
        path.end.z = 0

        o = path.start.to_arr()
        path_uvec = path.direction().to_arr()
        d = path.length()
        c = self.position.to_arr()
        r = self.size[0]

        s = np.dot(path_uvec, o - c) ** 2 - (np.linalg.norm(o - c) ** 2 - r ** 2)

        if s < 0:
            return True
        else:
            dist = np.array([-np.dot(path_uvec, o - c) + np.sqrt(s), -np.dot(path_uvec, o - c) - np.sqrt(s)])
            if np.min(dist) > d:
                return True
            else:
                return False
