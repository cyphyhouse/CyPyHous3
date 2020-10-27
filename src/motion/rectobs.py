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

    def __repr__(self) -> str:
        return "RectObs(point=%s, size=%s)" % (repr(self.position), repr(self.size))

    def _isdisjoint_point(self, point: Pos) -> bool:
        assert all(self.size >= 0)
        obs_maxes = self.position.mk_arr() + (self.size/2)
        obs_mins = self.position.mk_arr() - (self.size/2)
        p = point.mk_arr()
        # Check that p is not inside the obstacle, i.e., out side the interval at any dimension
        return np.any(p < obs_mins) or np.any(obs_maxes < p)

    def _isdisjoint_seg(self, orig_path: Seg) -> bool:
        """ The path has some points inside the rectangle rect iff there exists eps <= 1 s.t.
                for all dimensions. rect.mins <= start + vector * eps <= rect.maxes.

        The negation which is the return result becomes
            For all eps. not (0 <= eps <= 1) or not (for all dims. rect.mins <= start + vector * eps <= rect.maxes)
        <=> For all eps. (for all dims. rect.mins <= vector * eps + start <= rect.maxes) ==> (eps<0 or 1<eps)
        We therefore collect the constraints from all dimensions and check following cases
        + the constraints implies eps < 0 or 1 < eps, i.e, the bounding box of path is disjoint with rect
        + no solution to eps, i.e., vacuously true

        Notes
        -----
        Rewrite the formula to prefer relational operators to arithmetic operators
        because arithmetic operators may cause precision loss
        """
        assert all(self.size >= 0)
        if not self._isdisjoint_point(orig_path.start) or \
                not self._isdisjoint_point(orig_path.end):
            # Early return because one of two ends is inside the obstacle
            return False

        obs_maxes = self.position.mk_arr() + (self.size/2)
        obs_mins = self.position.mk_arr() - (self.size/2)

        start = orig_path.start.mk_arr()
        end = orig_path.end.mk_arr()
        bbox_mins = np.minimum(start, end)
        bbox_maxes = np.maximum(start, end)
        # Below directly implies eps < 0 or 1 < eps
        if np.any(obs_maxes < bbox_mins) or np.any(bbox_maxes < obs_mins):
            return True
        # else: eps has no solution, vacuously true
        # Collect currently tightest min and max for eps separately while avoid division
        # FIXME avoid division if possible
        curr_eps_lo, curr_eps_hi = -np.inf, np.inf
        for vec_i, min_i, max_i in zip(end - start, obs_mins - start, obs_maxes - start):
            if vec_i > 0:
                curr_eps_lo = max(curr_eps_lo, min_i / vec_i)
                curr_eps_hi = min(curr_eps_hi, max_i / vec_i)
            elif vec_i < 0:
                curr_eps_lo = max(curr_eps_lo, max_i / vec_i)
                curr_eps_hi = min(curr_eps_hi, min_i / vec_i)
            elif not (min_i <= 0 <= max_i):  # vec_i == 0
                return True

            if curr_eps_lo > curr_eps_hi:  # no solution to eps
                return True
        return False


def cross_product(v: np.ndarray, w: np.ndarray) -> float:
    return v[0]*w[1] - v[1]*w[0]
