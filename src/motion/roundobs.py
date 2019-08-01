import numpy as np
import math

from src.motion.obstacle import Obstacle
from src.motion.pos_types import Pos, Seg


class RoundObs(Obstacle):

    def __init__(self, point, radius):
        super(RoundObs, self).__init__(point, np.array([radius]))

    def _collision_point(self, point: Pos) -> bool:
        print("collision point")
        d = (self.position.x - point.x) ** 2 + (self.position.y - point.y) ** 2 + (self.position.z - point.z) ** 2

        return math.sqrt(d) > self.size[0]

    def _collision_path(self, path: Seg) -> bool:
        print("collision path")
        ab = np.array([path.vector.x, path.vector.y, path.vector.z])
        ac = np.array([self.position.x - path.start.x, self.position.y - path.start.y, self.position.z - path.start.z])

        d = np.linalg.norm(np.cross(ac, ab)) / np.linalg.norm(ab)

        return d > self.size[0]


'''
p1 = Pos(np.array([-1, 0, 1]))
p2 = Pos(np.array([1, 0, 1]))
path = Seg(p1,p2)

o1 = RoundObs(Pos(np.array([0,0.1,1])), 0.5)

b = o1.collision_check(path)
print(b)
'''