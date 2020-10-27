import numpy as np
import math

from src.motion.obstacle import Obstacle
from src.motion.pos_types import Pos, Seg


class RoundObs(Obstacle):

    def __init__(self, point, radius):
        super(RoundObs, self).__init__(point, np.array([radius]))

    @property
    def radius(self) -> float:
        return self.size[0]

    def _isdisjoint_point(self, point: Pos) -> bool:
        d = (self.position.x - point.x) ** 2 + (self.position.y - point.y) ** 2 + (self.position.z - point.z) ** 2

        return math.sqrt(d) > self.radius

    def _isdisjoint_seg(self, path: Seg) -> bool:
        o = np.array([path.start.x, path.start.y, path.start.z])
        path_uvec = path.direction().mk_arr()
        d = path.length()
        c = self.position.mk_arr()
        r = self.radius

        s = np.dot(path_uvec, o - c) ** 2 - ( np.linalg.norm(o - c) ** 2 - r ** 2 )

        if s < 0:
            return True
        else:
            dist = np.array([-np.dot(path_uvec, o - c) + np.sqrt(s), -np.dot(path_uvec, o - c) - np.sqrt(s)])
            if np.min(dist) > d:
                return True
            else:
                return False


'''
p1 = Pos(np.array([-1, 0, 1]))
p2 = Pos(np.array([0.75, 0, 1]))
path = Seg(p1, p2)

o1 = RoundObs(Pos(np.array([0, 0, 1])), 0.5)

b = o1.collision_check(path)
print(b)
'''