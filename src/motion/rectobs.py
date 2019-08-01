import numpy as np
import math

from src.motion.obstacle import Obstacle
from src.motion.pos_types import Pos, Seg


class RectObs(Obstacle):

    def __init__(self, point, size):
        """
        :param point: center of rectangle box
        :param size: length (x), width (y), height (z) of the box
        """
        super(RectObs, self).__init__(point, size)

    def _collision_point(self, point: Pos) -> bool:
        d = point - self.position
        return math.fabs(d.x) > self.size[0]/2 or math.fabs(d.y) > self.size[1]/2 or math.fabs(d.z) > self.size[2]/2

    def _collision_path(self, path: Seg) -> bool:
        # TODO


        return False


'''
p1 = Pos(np.array([-1, 0, 1]))
p2 = Pos(np.array([1, 0, 1]))
path = Seg(p1, p2)

o1 = RectObs(Pos(np.array([-0.5, 0, 1])), np.array([1, 1, 1]))

b = o1.collision_check(p1)
print(b)
'''