# Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

import typing as tp

import src.datatypes.motion.pos as pos
import src.datatypes.motion.seg as seg
import src.motion.abstract.planner as planner


class SimplePlanner(planner.Planner):
    """
    simple path planner. returns a path consisting of straight line path segments from start position to goal position.
    __num_seg : number of segments in a path
    """

    def __init__(self, num_seg=1):
        """
        :param num_seg: number of segments
        :type num_seg: int
        """
        super(SimplePlanner, self).__init__()
        self.__num_seg = num_seg

    # ------------ MEMBER ACCESS METHODS --------------

    @property
    def num_seg(self) -> int:
        return self.__num_seg

    @num_seg.setter
    def num_seg(self, num_seg: int) -> None:
        self.__num_seg = num_seg

    def find_path(self, start_point: pos.Pos, end_point: pos.Pos,
                  obstacles: tp.Union[tp.List[pos.Pos], None] = None) -> list:
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

        dist = pos.distance(start_point, end_point)
        segment = seg.Seg(start_point, end_point)

        # direction is the unit vector along the line segment.

        direction = segment.direction()

        seg_length = dist / self.num_seg
        path = [start_point]
        last_point = start_point
        for i in range(0, self.num_seg - 1):
            last_point = (last_point + direction * seg_length)
            path.append(last_point)
        path.append(end_point)
        return path
