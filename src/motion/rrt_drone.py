"""
Path Planning Sample Code with RRT

Author: AtsushiSakai(@Atsushi_twi)

Modifications for use in CyPhyHouse made by Joao
"""

# TODO: revisit documentation.
import copy
import math
import random
from typing import Union

import numpy as np

from src.motion.planner import Planner
from src.motion.pos_types import Pos, Node, to_node, Seg


class RRT(Planner):
    """
    Class for RRT* Planning
    """

    def __init__(self, rand_area: list = None, expand_dis: float = 0.5, goal_sample_rate: int = 15,
                 max_iter: int = 200):
        super(RRT, self).__init__()
        if rand_area is None:
            rand_area = [-2.5, 2.5, -2.5, 2.5, 0.5, 2.5]
        self.min_xrand = rand_area[0]
        self.max_xrand = rand_area[1]
        self.min_yrand = rand_area[2]
        self.max_yrand = rand_area[3]
        self.min_zrand = rand_area[4]
        self.max_zrand = rand_area[5]
        self.expand_dis = expand_dis
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter

    def find_path(self, start: Pos, end: Pos, obstacle_list: Union[list, None] = None,
                  search_until_max_iter: bool = False) -> \
            Union[list, None]:
        """
        RRT* Path Planning
        search_until_max_iter: Search until max iteration for path improving or not
        """
        if obstacle_list is None:
            obstacle_list = []
        start = to_node(start)
        end = to_node(end)
        # print("Start ", start)
        # print("End", end)
        if end.z == 0:
            print("z = 0, point not valid for drone")
            return None

        self.node_list = [start]
        for i in range(self.max_iter):
            rnd = self.get_random_point(end)
            nind = self.get_nearest_list_index(rnd)

            new_node = self.steer(rnd, nind)
            node_path = Seg(self.node_list[nind], new_node)
            if self.check_collision(obstacle_list, node_path):
                self.node_list.append(new_node)

            if self.calc_dist_to_goal(end, new_node.x, new_node.y, new_node.z) <= self.expand_dis:
                path = self.gen_final_course(start, end, new_node)
                return self.path_smoothing(obstacle_list, path[::-1], 100)

        print("Reached max iteration")

        return None

    def steer(self, rnd: list, nind: int) -> Node:
        """
        steer vehicle.
        :param rnd:
        :param nind:
        :return:
        """
        # expand tree
        nearest_node = self.node_list[nind]

        seg_nn_to_rnd = Seg(nearest_node, Pos(np.array(rnd)))
        nn_to_rnd_uvec = seg_nn_to_rnd.direction()
        new_node = Node(rnd[0], rnd[1], rnd[2])

        # Find a point within expand_dis of nind, and closest to rnd
        if seg_nn_to_rnd.length() <= self.expand_dis:
            pass
        else:
            new_node.x = nearest_node.x + self.expand_dis * nn_to_rnd_uvec.x
            new_node.y = nearest_node.y + self.expand_dis * nn_to_rnd_uvec.y
            new_node.z = nearest_node.z + self.expand_dis * nn_to_rnd_uvec.z
        new_node.parent = nearest_node
        return new_node

    def get_random_point(self, end: Pos) -> list:
        """
        function to get a random point near the end
        :param end:
        :return:
        """
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = [random.uniform(self.min_xrand, self.max_xrand),
                   random.uniform(self.min_yrand, self.max_yrand),
                   random.uniform(self.min_zrand, self.max_zrand)]
        else:  # goal point sampling
            rnd = [end.x, end.y, end.z]

        return rnd

    def check_collision(self, obstacle_list: list, dir_seg: Seg):
        """
        extended check collision
        :param obstacle_list:
        :param dir_seg:
        :return:
        """
        for obs in obstacle_list:
            if not obs.collision_check(dir_seg):
                return False

        return True

    def get_nearest_list_index(self, rnd: list) -> int:
        """
        function for returning the list index of the nearest point
        :param rnd:
        :return:
        """
        dlist = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1]) ** 2 + (node.z - rnd[2]) ** 2 for node in self.node_list]
        minind = dlist.index(min(dlist))
        return minind

    def gen_final_course(self, start: Node, end: Node, last_node) -> list:
        """
        generate the final path
        :param start:
        :param end:
        :param last_node:
        :return:
        """
        path = [Pos(np.array([end.x, end.y, end.z]))]

        node = last_node
        while node.parent is not None:
            path.append(Pos(np.array([node.x, node.y, node.z])))
            node = node.parent
        path.append(Pos(np.array([start.x, start.y, start.z])))
        return path

    def calc_dist_to_goal(self, end: Node, x: float, y: float, z: float) -> float:
        """
        calculate the distance to goal
        :param end:
        :param x:
        :param y:
        :param z:
        :return:
        """
        return np.linalg.norm([x - end.x, y - end.y, z - end.z])

    def path_smoothing(self, obstacle_list: list, path: list, max_iter: int):
        """
        smooth path
        :param path:
        :return:
        """

        for _ in range(max_iter):
            path_len = len(path)
            pickPoints = [random.randint(0, path_len-1), random.randint(0, path_len-1)]
            if not (pickPoints[0] == pickPoints[1]):
                pickPoints.sort()
                if not (pickPoints[0]+1 == pickPoints[1]):
                    sub_seg = Seg(path[pickPoints[0]], path[pickPoints[1]])
                    if not self.check_collision(obstacle_list, sub_seg):
                        continue
                    path = path[0:pickPoints[0]+1] + path[pickPoints[1]:]
        return path


'''
a = RRT()
p1 = Pos(np.array([-2, 0, 1]))
p2 = Pos(np.array([2, 0, 1]))

from src.motion.cylobs import CylObs
o1 = CylObs(Pos(np.array([0, 0, 0])), 0.5)

import time
loops = 100
start_time = time.time()
for i in range(loops):
    c = a.find_path(p1, p2, [o1])
elapsed_time = time.time() - start_time
print(elapsed_time/loops)
print(c)
'''
