# Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

from abc import ABC, abstractmethod

import numpy as np

import src.datatypes.motion.pos as pos
import src.datatypes.motion.seg as seg


class Obstacle(ABC):
    """
    abstract obstacle class.
    __position : position of the obstacle
    __size : dimensions, centered at position
    """

    def __init__(self, point: pos.Pos, size: np.ndarray):
        """
        initialize obs object
        :param point : position of the obstacle
        :type point : pos.Pos
        :param size : dimensions of the obstacle, centered at position
        :type size : np.ndarray
        """

        self.__position = point
        self.__size = size

    @property
    def position(self) -> pos.Pos:
        """
        getter method for position
        :return: position of obstacle
        """
        return self.__position

    @position.setter
    def position(self, pos: pos.Pos) -> None:
        """
        setter method for position
        :param pos: position
        :return: nothing
        """
        self.__position = pos

    @property
    def size(self) -> np.ndarray:
        """
        getter method for size"
        :return:
        """
        return self.__size

    @size.setter
    def size(self, size: np.ndarray) -> None:
        """
        setter method for size
        :param np.ndarray: size
        :return:
        """
        self.__size = size

    def collision_check(self, obj) -> bool:
        """
        Collision check method
        return true if safe, false if there is a collision
        """
        if isinstance(obj, pos.Pos):
            return self._collision_point(obj)
        elif isinstance(obj, seg.Seg):
            return self._collision_path(obj)
        return True

    @abstractmethod
    def _collision_point(self, point: pos.Pos) -> bool:
        """
        Collision check method for points
        """
        pass

    @abstractmethod
    def _collision_path(self, path: seg.Seg) -> bool:
        """
        Collision check method for paths
        """
        pass
