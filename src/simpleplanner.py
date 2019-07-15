import numpy as np
import planner as pl
from pos import Pos, Seg, distance


class SimplePlanner(pl.Planner):
    """
    simple path planner. return a path consisting of straight line path segments from start position to goal position.
    __num_segs : number of segments in a path
    """

    def __init__(self, num_segs=1):
        """
        initialize with number of segments
        :param num_segs: number of segments
        """
        super(SimplePlanner, self).__init__()
        self.__num_segs = num_segs
        pass

    @property
    def num_segs(self) -> int:
        """
        getter method for number of path segments returned by the planner
        :return: integer number of segments
        """
        return self.__num_segs

    @num_segs.setter
    def num_segs(self, num_segs: int) -> None:
        """
        setter method for number of path segments
        :param num_segs: integer number of segments
        :return: Nothing
        """
        self.__num_segs = num_segs

    def find_path(self, start_point: Pos, end_point: Pos) -> list:
        """
        find path of length num_segs between start and end point
        :param start_point: vector starting point
        :param end_point: vector end point
        :return: path
        """
        # calculating the unit vector along the line
        dist = distance(start_point, end_point)
        segment = Seg(start_point, end_point)
        direction = segment.direction()
        seg_length = dist / self.num_segs
        path = [start_point]
        last_point = start_point
        for i in range(0, self.num_segs - 1):
            last_point = (last_point + direction * seg_length)
            path.append(last_point)
        path.append(end_point)
        return path


x = Pos(np.array([1, 1, 1]))
y = Pos(np.array([2, 2, 0]))
z = SimplePlanner(2)
path = z.find_path(x, y)
print(path)
