from src.motion.planner import Planner
from src.motion.pos import Pos, Seg, distance


class SimplePlanner(Planner):
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

    def find_path(self, start_point: Pos, end_point: Pos, obstacles: list = None) -> list:
        """
        find path of length num_segs between start and end point
        :param start_point: vector starting point
        :param end_point: vector end point
        :param obstacles: obstacles
        :return: path
        """
        # calculating the unit vector along the line
        dist = distance(start_point, end_point)
        segment = Seg(start_point, end_point)
        direction = segment.direction()  # direction is the unit vector along the line segment.
        seg_length = dist / self.num_segs
        path = [start_point]
        last_point = start_point
        for i in range(0, self.num_segs - 1):
            last_point = (last_point + direction * seg_length)
            path.append(last_point)
        path.append(end_point)
        return path
