# Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

import math
import typing

import numpy as np
from scipy.spatial.distance import pdist


class Pos(object):
    """
    vector object, 3d point. Has x, y and z components, and possibly an orientation (yaw)
    __x : x coordinate
    __y : y coordinate
    __z : z coordinate
    __yaw : yaw
    """

    def __init__(self, vector: np.ndarray = np.array([])):
        """
        initialize 3d pointer given an np array
        :param vector: np array of length 3.
        :type vector: np.ndarray
        """
        try:
            self.__x = vector[0]
            self.__y = vector[1]
            self.__z = vector[2]
            if len(vector) == 4:
                self.__yaw = vector[3]
            elif len(vector) == 3:
                self.__yaw = 0.0

        except IndexError:
            print("indexerror, initializing an empty position")
            self.__x = None
            self.__y = None
            self.__z = None
            self.__yaw = None

    # ------------ MEMBER ACCESS METHODS --------------

    @property
    def x(self) -> float:
        """
        getter method for x coordinate
        :return: x coordinate
        """
        return self.__x

    @x.setter
    def x(self, x: float) -> None:
        """
        setter method for x coordinate
        :param x: float x coordinate
        :return: nothing
        """
        self.__x = x

    @property
    def y(self) -> float:
        """
        getter method for y coordinate
        :return: y coordinate
        """
        return self.__y

    @y.setter
    def y(self, y: float) -> None:
        """
        setter method for y coordinate
        :param y: float y coordinate
        :return: nothing
        """
        self.__y = y

    @property
    def z(self) -> float:
        """
        getter method for z coordinate
        :return: z coordinate
        """
        return self.__z

    @z.setter
    def z(self, z: float) -> None:
        """
        setter method for z coordinate
        :param z: float z coordinate
        :return: nothing
        """
        self.__z = z

    @property
    def yaw(self) -> float:
        """
        getter method for yaw
        :return: yaw
        """
        return self.__yaw

    @yaw.setter
    def yaw(self, yaw: float) -> None:
        """
        setter method for yaw
        :param yaw: float yaw
        :return: nothing
        """
        self.__yaw = yaw

    def __repr__(self) -> str:
        """
        string representation
        :return: string
        """
        return "<" + str(self.x) + "," + str(self.y) + "," + str(self.z) + ">"

    # ----------------- OPERATIONS --------------------

    def __eq__(self, other):
        """
        comparision operator
        :param other: other vector
        :return: true or false based on whether the other position is the same
        """
        return self.x == other.x and self.y == other.y and self.z == other.z

    def __add__(self, other):
        """
        addition
        :param other: vector to add to
        :return: vector
        """
        return Pos(np.array(self.to_arr() + other.to_arr()))

    def __sub__(self, other):
        """
        subtraction
        :param other: vector to subtract
        :return: vector
        """
        return Pos(np.array(self.to_arr() - other.to_arr()))

    def __mul__(self, other: typing.Union[int, float]):
        """
        scalar multiplication
        :param other: scalar
        :return: vector
        """
        return Pos(np.array(self.to_arr() * other))

    def __rmul__(self, other: typing.Union[int, float]):
        """
        scalar multiplication, commutative
        :param other: scalar
        :return: vector
        """
        return self.__mul__(other)

    # ----------------- PROPERTIES --------------------

    def magnitude(self) -> float:
        """
        magnitude of the vector, distance from origin
        :return: float distance of origin
        """
        X = np.vstack((np.array([0, 0, 0]), self.to_arr()))
        return pdist(X)[0]

    def direction(self):
        """
        unit vector of direction
        :return: Position
        """
        len = math.sqrt(self.x ** 2 + self.y ** 2 + self.z ** 2)
        uvec = Pos(np.transpose(np.array([self.x / len,
                                          self.y / len,
                                          self.z / len])))
        return uvec

    # -------------  CONVERSION METHODS ---------------

    def to_arr(self) -> np.ndarray:
        """
        method to make into np array
        :return: point in array format.
        """
        return np.array([self.x, self.y, self.z])

    def to_pose(self):
        """
        convert position to posestamped
        :return:
        """
        from geometry_msgs.msg import Pose
        return_pt = Pose()

        return_pt.position.x = self.x
        return_pt.position.y = self.y
        return_pt.position.z = self.z

        return return_pt

    def to_list(self) -> typing.List[float]:
        """
        :return: list
        """
        return [self.x, self.y, self.z]


# --------- POS FUNCTIONS -------------

def distance(x: Pos, y: Pos) -> float:
    """
    distance between two points
    :param x: position
    :param y: position
    :return: distance
    """
    d = np.vstack((x.to_arr(), y.to_arr()))
    dist = pdist(d)[0]
    return dist
