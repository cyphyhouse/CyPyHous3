# Copyright (c) 2019 CyPhyHouse. All Rights Reserved.


from abc import ABC, abstractmethod

from src.datatypes.motion.pos import Pos


class Planner(ABC):
    """
    abstract planner class. should have a find_path method.
    """

    def __init__(self):
        """
        default abstract class, maybe extended later
        """
        pass

    @abstractmethod
    def find_path(self, start_point: Pos, end_point: Pos, obstacles: list) -> list:
        """
        find a path from the start point to an end point given a list of obstacles.
        :param start_point: starting point
        :param end_point: ending point
        :param obstacles: list of obstacles.
        :return: path
        """
        pass
