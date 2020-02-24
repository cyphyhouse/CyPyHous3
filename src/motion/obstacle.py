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

    def collision_check(self, obj) -> bool:
        """
        Collision check method
        return true if safe, false if there is a collision
        """
        if isinstance(obj, Pos):
            return self._collision_point(obj)
        elif isinstance(obj, Seg):
            return self._collision_path(obj)
        return True

    @abstractmethod
    def _collision_point(self, point: Pos) -> bool:
        """
        Collision check method for points
        """
        pass

    @abstractmethod
    def _collision_path(self, path: Seg) -> bool:
        """
        Collision check method for paths
        """
        pass

    def __eq__(self, other):
        return self.position == other.position and self.size.all() == other.size.all()
