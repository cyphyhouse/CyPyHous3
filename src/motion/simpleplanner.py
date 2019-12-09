# Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

from typing import List, Union

from src.datatypes.motion.pos import Pos, distance
from src.datatypes.motion.seg import Seg
from src.motion.abstract.planner import Planner


class SimplePlanner(Planner):
    """
    simple path planner. returns a path consisting of straight line path segments from start position to goal position.
    __num_seg : number of segments in a path
    """

    def __init__(self, num_seg=1):
        """
        initialize with number of segments
        :param num_seg: number of segments
        :type num_seg: int
        """
        super(SimplePlanner, self).__init__()
        self.__num_seg = num_seg

    @property
    def num_seg(self) -> int:
        """
        getter method for number of path segments returned by the planner
        :return: integer number of segments
        """
        return self.__num_seg

    @num_seg.setter
    def num_seg(self, num_seg: int) -> None:
        """
        setter method for number of path segments
        :param num_seg: integer number of segments
        :return: Nothing
        """
        self.__num_seg = num_seg

    def find_path(self, start_point: Pos, end_point: Pos, obstacles: Union[List[Pos], None] = None) -> list:
        """
        find path of length num_seg between start and end point
        :param start_point: starting point
        :type start_point: Pos
        :param end_point: end point
        :type end_point: Pos
        :param obstacles: obstacles
        :type obstacles: List
        :return: path
        """

        # TODO: write simple planner to compute path while avoiding obstacles with a given segment length.

        if obstacles is None:
            obstacles = []
        # calculating the unit vector along the line
        dist = distance(start_point, end_point)
        segment = Seg(start_point, end_point)

        direction = segment.direction()  # direction is the unit vector along the line segment.
        seg_length = dist / self.num_seg
        path = [start_point]
        last_point = start_point
        for i in range(0, self.num_seg - 1):
            last_point = (last_point + direction * seg_length)
            path.append(last_point)
        path.append(end_point)
        return path
