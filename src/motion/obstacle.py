import typing
from abc import ABC, abstractmethod
import numpy as np

from src.motion.pos_types import Pos, Seg, Node


class Obstacle(ABC):
    def __init__(self, point: Pos, size: np.ndarray):
        """
        initialize obs object
        """
        
        self.__position = point
        self.__size = size

    @property
    def position(self) -> Pos:
        """
        getter method for position"
        :return:
        """
        return self.__position

    @position.setter
    def position(self, pos: Pos) -> None:
        """
        setter method for position
        :param pos: position
        :return:
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

    def isdisjoint(self, obj) -> bool:
        """
        Use to check collision
        return true if disjoint, false if there is a collision
        """
        if isinstance(obj, Pos):
            return self._isdisjoint_point(obj)
        elif isinstance(obj, Seg):
            return self._isdisjoint_seg(obj)
        raise ValueError("Unsupported type %s for checking disjointness" % type(obj))

    @abstractmethod
    def _isdisjoint_point(self, point: Pos) -> bool:
        """
        Collision check method for points
        """
        raise NotImplementedError

    @abstractmethod
    def _isdisjoint_seg(self, path: Seg) -> bool:
        """
        Collision check method for paths
        """
        raise NotImplementedError

    def __eq__(self, other):
        return self.position == other.position and self.size == other.size()
