import numpy as np
import math
import copy

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

    def _collision_path(self, orig_path: Seg) -> bool:
        # TODO
        # Check whether the path intersects with any of the 4 edges of the obstacle (Only apply to cars)
        # Refered this algorithm from stackoverflow:
        #   https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect

        path = copy.deepcopy(orig_path)
        path.start.z = 0
        path.end.z = 0
        path.vector.z = 0

        p = path.start.mk_arr()
        r = path.vector.mk_arr() 

        # Find 4 Points
        a = Pos(np.array([self.position.x - self.size[0]/2, self.position.y - self.size[1]/2, 0]))
        b = Pos(np.array([self.position.x - self.size[0]/2, self.position.y + self.size[1]/2, 0]))
        c = Pos(np.array([self.position.x + self.size[0]/2, self.position.y - self.size[1]/2, 0]))
        d = Pos(np.array([self.position.x + self.size[0]/2, self.position.y + self.size[1]/2, 0]))

        # Find 4 edges 
        edges = []
        edges.append(Seg(a, b))
        edges.append(Seg(b, c))
        edges.append(Seg(c, d))
        edges.append(Seg(d, a))

        # Check collision
        for edge in edges:
            q = edge.start.mk_arr() 
            s = edge.vector.mk_arr() 

            r_x_s = cross_product(r, s)
            if r_x_s == 0.0:
                continue 
            
            t = cross_product((q-p), s)/r_x_s
            u = cross_product((q-p), r)/r_x_s

            if t <= 1.0 and t >= 0.0 and u <= 1.0 and u >= 0.0:
                return False


        return True

def cross_product(v: np.ndarray, w: np.ndarray) -> float:
    return v[0]*w[1] - v[1]*w[0]



p1 = Pos(np.array([1.5, 0, 1]))
p2 = Pos(np.array([1, 1, 1]))
path = Seg(p1, p2)

o1 = RectObs(Pos(np.array([0.5, 0.5, 0])), np.array([1, 1, 1]))

b = o1.collision_check(path)
print(b)
