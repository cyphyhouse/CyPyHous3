import typing
import numpy as np

from src.motion.pos_types import Pos, Seg, Node


class Obstacle(object):
    def __init__(self, point: Pos, size: np.ndarray):
        """
        initialize obs object
        """
        
        __position = point
        __size = size

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
        """
        if obj is Pos:
            return self.__collision_point(obj)
        elif obj is Seg:
            return self.__collision_path(obj)
        return False

    @abstractmethod
    def __collision_point(self, point: Pos) -> bool:
        """
        Collision check method for points
        """
        pass

    @abstractmethod
    def __collision_path(self, path: Seg) -> bool:
        """
        Collision check method for paths
        """
        pass
