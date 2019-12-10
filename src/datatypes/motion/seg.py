# Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

import numpy as np

import src.datatypes.motion.pos as pos

class Seg(object):
    """
    path segment object, specified by a start and an end position
    __start : start position
    __end : end position
    """

    def __init__(self, start: pos.Pos, end: pos.Pos):
        """
        initialization method
        :param start: start position
        :type start: pos.Pos
        :param end: end position
        :type end: pos.Pos
        """
        self.__start = start
        self.__end = end

    # ------------ MEMBER ACCESS METHODS --------------

    @property
    def start(self) -> pos.Pos:
        """
        setter method for starting point of segment
        :return: start pos
        """
        return self.__start

    @start.setter
    def start(self, start: pos.Pos) -> None:
        """
        setter method for start pos of segment
        :param start: start position
        :return: nothing
        """
        self.__start = start

    @property
    def end(self) -> pos.Pos:
        """
        setter method for ending point of segment
        :return: end pos
        """
        return self.__end

    @end.setter
    def end(self, end: pos.Pos) -> None:
        """
        setter method for end pos of segment
        :param end: end position
        :return: nothing
        """
        self.__end = end

    def __repr__(self) -> str:
        """
        string representation
        :return: string
        """
        return str(self.start) + ":" + str(self.end)

    # ----------------- PROPERTIES --------------------

    def length(self) -> float:
        """
        length of the segment
        :return: float distance between start and end point
        """
        return pos.distance(self.start, self.end)

    def direction(self):
        """
        unit vector of direction
        :return: Position
        """
        len = self.length()
        if len == 0.0:
            uvec = pos.Pos(np.transpose(np.array([0, 0, 0])))
        else:
            uvec = pos.Pos(np.transpose(np.array([(self.end.x - self.start.x) / len,
                                                  (self.end.y - self.start.y) / len,
                                                  (self.end.z - self.start.z) / len])))
        return uvec
