# Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

import abc
import typing as tp

import src.datatypes.motion.abstract.obstacle as obstacle
import src.datatypes.motion.pos as pos


class Planner(abc.ABC):
    """
    abstract planner class. should have a find_path method.
    """

    def __init__(self):
        """
        default abstract class, maybe extended later
        """
        pass

    @abc.abstractmethod
    def find_path(self, start_point: pos.Pos, end_point: pos.Pos,
                  obstacles: tp.Union[tp.List[obstacle.Obstacle], None]) -> tp.List[pos.Pos]:
        """
        find a path from the start point to an end point given a list of obstacles.

        :param start_point: starting point
        :type start_point: Pos

        :param end_point: ending point
        :type end_point: Pos

        :param obstacles: list of obstacles.
        :type obstacles: List[Obstacle]

        :return: path
        :rtype: List[Pos]
        """
        pass
