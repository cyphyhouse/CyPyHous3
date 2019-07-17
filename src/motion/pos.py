import typing

import numpy as np
from scipy.spatial.distance import pdist


class Pos(object):
    """
    vector object, 3d point. Has x, y and z components.
    """

    def __init__(self, vector: np.ndarray = np.array([])):
        """
        initialize 3d pointer given an np array
        :param vector: np array of length 3.
        """
        try:
            self.x = vector[0]
            self.y = vector[1]
            self.z = vector[2]
        except IndexError:
            print("initializing an empty position")
            self.x = None
            self.y = None
            self.z = None

    def __repr__(self) -> str:
        """
        string representation
        :return: string
        """
        return "<" + str(self.x) + "," + str(self.y) + "," + str(self.z) + ">"

    def mk_arr(self) -> np.ndarray:
        """
        method to make into np array
        :return: point in array format.
        """
        return np.array([self.x, self.y, self.z])

    def magnitude(self) -> float:
        """
        magnitude of the vector. distance from origin
        :return: float distance of origin
        """
        X = np.vstack((np.array([0, 0, 0]), self.mk_arr()))
        return pdist(X)[0]

    def __add__(self, other):
        """
        addition
        :param other: vector to add to
        :return: vector
        """
        return Pos(np.array(self.mk_arr() + other.mk_arr()))

    def __sub__(self, other):
        """
        subtraction
        :param other: vector to subtract
        :return: vector
        """
        return Pos(np.array(self.mk_arr() - other.mk_arr()))

    def __mul__(self, other: typing.Union[int, float]):
        """
        scalar multiplication
        :param other: scalar
        :return: vector
        """
        return Pos(np.array(self.mk_arr() * other))

    def __eq__(self, other):
        """
        comparision operator
        :param other: other vector
        :return: true or false based on whether the other position is the same
        """
        return self.x == other.x and self.y == other.y and self.z == other.z

    def to_pose(self):
        """
        convert position to posestamped
        :return:
        """
        from geometry_msgs.msg import PoseStamped, Pose
        return_pt = Pose()
        return_pt.position.x = self.x
        return_pt.position.y = self.y
        return_pt.position.y = self.z


class Seg(object):
    """
    path segment object, created from a start and an end position
    """

    def __init__(self, start: Pos, end: Pos):
        """
        initialization method
        :param start: start position
        :param end: end position
        """
        self.start = start
        self.end = end

    def __repr__(self) -> str:
        """
        string representation
        :return: string
        """
        return str(self.start) + ":" + str(self.end)

    def length(self) -> float:
        """
        length of the segment
        :return: float distance between start and end point
        """
        return distance(self.start, self.end)

    def direction(self):
        """
        unit vector of direction
        :return: Position
        """
        uvec = Pos(np.transpose(np.array([(self.end.x - self.start.x) / self.length(),
                                          (self.end.y - self.start.y) / self.length(),
                                          (self.end.z - self.start.z) / self.length()])))
        return uvec





def distance(x: Pos, y: Pos) -> float:
    """
    distance between two points
    :param x: position
    :param y: position
    :return: distance
    """
    d = np.vstack((x.mk_arr(), y.mk_arr()))
    dist = pdist(d)[0]
    return dist


def dot(x: Pos, y: Pos) -> np.ndarray:
    """
    dot product
    :param x: vector (position)
    :param y: vector (position)
    :return: dot product
    """
    return np.dot(x.mk_arr(), y.mk_arr())


def cross(x: Pos, y: Pos) -> Pos:
    """
    cross product
    :param x: vector
    :param y: vector
    :return: vector
    """
    return Pos(np.cross(x.mk_arr(), y.mk_arr()))