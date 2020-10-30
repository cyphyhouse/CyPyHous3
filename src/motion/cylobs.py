import numpy as np
import math
import copy

from src.motion.obstacle import Obstacle
from src.motion.pos_types import Pos, Seg


class CylObs(Obstacle):

    def __init__(self, point: Pos, radius: float, height: float = 1.0):
        point.z = 0
        super(CylObs, self).__init__(point, np.array([2*radius, 2*radius, height]))

    @property
    def diameter(self) -> float:
        assert self.size[0] == self.size[1]
        return self.size[0]

    @property
    def radius(self) -> float:
        return self.diameter/ 2

    @property
    def height(self) -> float:
        return self.size[2]

    def _isdisjoint_point(self, point: Pos) -> bool:
        d = (self.position.x - point.x) ** 2 + (self.position.y - point.y) ** 2

        return math.sqrt(d) > self.radius

    def _isdisjoint_seg(self, orig_path: Seg) -> bool:
        path = copy.deepcopy(orig_path)
        path.start.z = 0
        path.end.z = 0
        path.vector.z = 0

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
p1 = Pos(np.array([-1.03, -0.1566, 1.10]))
p2 = Pos(np.array([1.094, 0.29, 1.22]))
path = Seg(p1, p2)

o1 = CylObs(Pos(np.array([0, 0, 0])), 0.5)

b = o1.collision_check(path)
print(b)
'''