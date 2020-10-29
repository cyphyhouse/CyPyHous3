# abstract class for path planner
# use numpy arrays

from abc import ABC, abstractmethod
from typing import Optional, Sequence, Tuple

from scipy.spatial import Rectangle

from src.motion.pos_types import Pos, pos3d
from src.motion.cylobs import Obstacle


class Planner(ABC):
    """
    abstract planner class. should have a find_path method.
    """

    def __init__(self, rand_area: Tuple[Pos, Pos] = ()):
        """
        initialization method
        """
        if not rand_area:
            rand_area = (pos3d(-10, -10, 0),
                         pos3d(10, 10, 5))
        self._rand_area = Rectangle(mins=rand_area[0].mk_arr(),
                                    maxes=rand_area[1].mk_arr())

    @property
    def min_rand(self) -> float:
        return min(self._rand_area.mins[0:2])

    @min_rand.setter
    def min_rand(self, value: float) -> None:
        self._rand_area.min_xrand = value
        self._rand_area.min_yrand = value

    @property
    def max_rand(self) -> float:
        return max(self._rand_area.maxes[0:2])

    @max_rand.setter
    def max_rand(self, value: float) -> None:
        self._rand_area.max_xrand = value
        self._rand_area.max_yrand = value

    @property
    def min_xrand(self) -> float:
        return self._rand_area.mins[0]

    @min_xrand.setter
    def min_xrand(self, value: float) -> None:
        self._rand_area.mins[0] = value

    @property
    def max_xrand(self) -> float:
        return self._rand_area.maxes[0]

    @max_xrand.setter
    def max_xrand(self, value: float) -> None:
        self._rand_area.maxes[0] = value

    @property
    def min_yrand(self) -> float:
        return self._rand_area.mins[1]

    @min_yrand.setter
    def min_yrand(self, value: float) -> None:
        self._rand_area.mins[1] = value

    @property
    def max_yrand(self) -> float:
        return self._rand_area.maxes[1]

    @max_yrand.setter
    def max_yrand(self, value: float) -> None:
        self._rand_area.maxes[1] = value

    @property
    def min_zrand(self) -> float:
        return self._rand_area.mins[2]

    @min_zrand.setter
    def min_zrand(self, value: float) -> None:
        self._rand_area.mins[2] = value

    @property
    def max_zrand(self) -> float:
        return self._rand_area.maxes[2]

    @max_zrand.setter
    def max_zrand(self, value: float) -> None:
        self._rand_area.maxes[2] = value

    @abstractmethod
    def find_path(self, start_point: Pos, end_point: Pos, obstacles: Sequence[Obstacle]) -> Sequence[Pos]:
        """
        find a path from the start point to an end point given a list of obstacles.
        :param start_point: starting point as a Pos [x,y,z]
        :param end_point: ending point as a Pos
        :param obstacles: as sequence of obstacles.
        :return: path as a sequence of Pos.
                 Empty sequence if no path is found.
        """
        raise NotImplementedError
