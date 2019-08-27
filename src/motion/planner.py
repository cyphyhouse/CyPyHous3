# abstract class for path planner
# use numpy arrays

from abc import ABC, abstractmethod

from src.motion.pos_types import Pos


class Planner(ABC):
    """
    abstract planner class. should have a find_path method.
    """

    def __init__(self):
        """
        initialization method
        """
        pass

    @abstractmethod
    def find_path(self, start_point: Pos, end_point: Pos, obstacles: list) -> list:
        """
        find a path from the start point to an end point given a list of obstacles.
        :param start_point: starting point as a vector [x,y,z]
        :param end_point: ending point as a vector
        :param obstacles: list of obstacles.
        :return: path as a list of vectors
        """
        pass


