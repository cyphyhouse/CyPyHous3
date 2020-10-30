"""
Path Planning Sample Code with RRT and Dubins path

author: AtsushiSakai(@Atsushi_twi)
Modifications for use in CyPhyHouse made by Ritwika Ghosh and Amelia Gosse

"""
# TODO: Complete documentation

import copy
import math
import random
from typing import Optional, Sequence, MutableSequence
import numpy as np

import src.motion.dubins_path_planning as dubins_path_planning
from src.motion.obstacle import Obstacle
from src.motion.pos_types import Node, to_node, Pos


class RRT_DUBINS():
    """
    Class for RRT* Planning with Dubins Car Dynamics
    """

    def __init__(self, rand_area: Sequence[float] = (), expand_dis: float = 0.1, goal_sample_rate: int = 15,
                 max_iter: int = 100):
        super(RRT_DUBINS, self).__init__()
        if not rand_area:
            rand_area = [-2.5, 2.5]
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.expand_dis = expand_dis
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter

    def find_path(self, start_point: Pos, end_point: Pos, obstacle_list: Sequence[Obstacle]) \
            -> Optional[Sequence[Pos]]:
        """
        Path Planning
        :param start_point: initial position of vehicle
        :param end_point: desired final position of vehicle
        :param obstacle_list: list of obstacles for planner to avoid
        :return: planned path
        """
        start_point = to_node(start_point)
        end_point = to_node(end_point)

        node_list = [start_point]
        for i in range(self.max_iter):
            rnd = self.get_random_point(end_point)
            nind = self.get_nearest_list_index(node_list, rnd)

            new_node = self.steer(rnd, nind, node_list)
            #  print(new_node.cost)

            if self.collision_check(new_node, obstacle_list):
                nearinds = self.find_near_nodes(new_node, node_list)
                new_node = self.choose_parent(new_node, nearinds, node_list, obstacle_list)
                node_list.append(new_node)
                self.rewire(nearinds, node_list, obstacle_list)

        # generate course
        lastIndex = self.get_best_last_index(end_point, node_list)
        #  print(lastIndex)

        if lastIndex is None:
            return None

        path = self.gen_final_course(start_point, end_point, lastIndex, node_list)
        return path

    def choose_parent(self, newNode: Node, nearinds: Sequence[int],
                      node_list: Sequence[Node], obstacle_list: Sequence[Obstacle]) -> Node:
        """

        :param newNode:
        :param nearinds:
        :param node_list:
        :param obstacle_list:
        :return:
        """
        if not nearinds:
            return newNode

        dlist = []
        for i in nearinds:
            tNode = self.steer(newNode, i, node_list)
            if self.collision_check(tNode, obstacle_list):
                dlist.append(tNode.cost)
            else:
                dlist.append(float("inf"))

        mincost = min(dlist)
        minind = nearinds[dlist.index(mincost)]

        if mincost == float("inf"):
            print("mincost is inf")
            return newNode

        newNode = self.steer(newNode, minind, node_list)

        return newNode

    def pi_2_pi(self, angle: float) -> float:
        """

        :param angle:
        :return:
        """
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def steer(self, rnd: Node, nind: int, node_list: Sequence[Node]) -> Node:
        """

        :param rnd:
        :param nind:
        :param node_list:
        :return:
        """
        #  print(rnd)
        curvature = 1.0

        nearestNode = node_list[nind]

        px, py, pyaw, path, mode, clen = dubins_path_planning.dubins_path_planning(
            nearestNode.x, nearestNode.y, nearestNode.yaw, rnd.x, rnd.y, rnd.yaw, curvature)

        newNode = copy.deepcopy(nearestNode)
        newNode.x = px[-1]
        newNode.y = py[-1]
        newNode.yaw = pyaw[-1]

        newNode.path_x = px
        newNode.path_y = py
        newNode.path_yaw = pyaw
        newNode.cost += clen
        newNode.parent = nind

        return newNode

    def get_random_point(self, end_point: Node) -> Node:
        """

        :param end_point:
        :return:
        """
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = [random.uniform(self.min_rand, self.max_rand),
                   random.uniform(self.min_rand, self.max_rand),
                   random.uniform(-math.pi, math.pi)
                   ]
        else:  # goal point sampling
            rnd = [end_point.x, end_point.y, end_point.yaw]

        node = Node(rnd[0], rnd[1], rnd[2])

        return node

    def get_best_last_index(self, end_point: Node, node_list: Sequence[Node]) -> Optional[int]:
        """

        :param end_point:
        :param node_list:
        :return:
        """
        YAWTH = np.deg2rad(1.0)
        XYTH = 0.5

        goalinds = []
        for (i, node) in enumerate(node_list):
            if self.calc_dist_to_goal(node.x, node.y, end_point) <= XYTH:
                goalinds.append(i)

        # angle check
        fgoalinds = []
        for i in goalinds:
            if abs(node_list[i].yaw - end_point.yaw) <= YAWTH:
                fgoalinds.append(i)

        if not fgoalinds:
            return None

        mincost = min([node_list[i].cost for i in fgoalinds])
        for i in fgoalinds:
            if node_list[i].cost == mincost:
                return i

        return None

    def gen_final_course(self, start_point: Node, end_point: Node, goalind: int, node_list: Sequence[Node]) -> list:
        """

        :param start_point:
        :param end_point:
        :param goalind:
        :param node_list:
        :return:
        """
        path = [Pos(np.array([end_point.x, end_point.y, 0.0]))]
        while node_list[goalind].parent is not None:
            node = node_list[goalind]
            for (ix, iy) in zip(reversed(node.path_x), reversed(node.path_y)):
                path.append(Pos(np.array([ix, iy, 0.0])))
            #  path.append([node.x, node.y])
            goalind = node.parent
        path.append(Pos(np.array([start_point.x, start_point.y, 0.0])))
        return path

    def calc_dist_to_goal(self, x: float, y: float, end_point: Node) -> float:
        """

        :param x:
        :param y:
        :param end_point:
        :return:
        """
        return np.linalg.norm([x - end_point.x, y - end_point.y])

    def find_near_nodes(self, newNode: Node, node_list: Sequence[Node]) -> Sequence[int]:
        """

        :param newNode:
        :param node_list:
        :return:
        """
        nnode = len(node_list)
        r = 50.0 * math.sqrt((math.log(nnode) / nnode))
        #  r = self.expandDis * 5.0
        dlist = [(node.x - newNode.x) ** 2 +
                 (node.y - newNode.y) ** 2 +
                 (node.yaw - newNode.yaw) ** 2
                 for node in node_list]
        nearinds = [dlist.index(i) for i in dlist if i <= r ** 2]
        return nearinds

    def rewire(self, nearinds: Sequence[int], node_list: MutableSequence[Node],
               obstacle_list: Sequence[Obstacle]) -> None:
        """

        :param nearinds:
        :param node_list:
        :param obstacle_list:
        """
        nnode = len(node_list)

        for i in nearinds:
            nearNode = node_list[i]
            tNode = self.steer(nearNode, nnode - 1, node_list)

            obstacleOK = self.collision_check(tNode, obstacle_list)
            imporveCost = nearNode.cost > tNode.cost

            if obstacleOK and imporveCost:
                #  print("rewire")
                node_list[i] = tNode

    def get_nearest_list_index(self, node_list: Sequence[Node], rnd: Node) -> int:
        """

        :param node_list:
        :param rnd:
        :return:
        """
        dlist = []
        for node in node_list:
            if node is not None:
                dlist.append((node.x - rnd.x) ** 2 +
                             (node.y - rnd.y) ** 2 +
                             (node.yaw - rnd.yaw) ** 2)
        minind = dlist.index(min(dlist))

        return minind

    def collision_check(self, node: Node, obstacle_list: Sequence[Obstacle]) -> bool:
        """

        :param node:
        :param obstacle_list:
        :return:
        """
        for obstacle in obstacle_list:
            for (ix, iy) in zip(node.path_x, node.path_y):
                # FIXME checking collision against other types of obstacles
                # The check below is only for cylindrical obstacles
                dx = obstacle.x - ix
                dy = obstacle.y - iy
                d = dx * dx + dy * dy
                if d <= obstacle.radius ** 2:
                    return False  # collision

        return True  # safe


'''
def main():
    print("Start rrt star with dubins planning")

    # ====Search Path with RRT====
    obstacle_list = [
        RoundObs(5, 5, 1),
        RoundObs(3, 6, 2),
        RoundObs(3, 8, 2),
        RoundObs(3, 10, 2),
        RoundObs(7, 5, 2),
        RoundObs(9, 5, 2)
    ]  # [x,y,size(radius)]

    # Set Initial parameters
    start = Node(0.0, 0.0, np.deg2rad(0.0))
    goal = Node(10.0, 10.0, np.deg2rad(0.0))

    rrt = RRT_DUBINS(rand_area=[-2.0, 2.0])
    path = rrt.find_path(start, goal, obstacle_list)
    print(path)


if __name__ == '__main__':
    main()
'''
