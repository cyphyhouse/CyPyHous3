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
                 max_iter: int = 2000):
        super(RRT, self).__init__()
        if rand_area is None:
            rand_area = [-2.5, 2.5, -2.5, 2.5]
        self.min_xrand = rand_area[0]
        self.max_xrand = rand_area[1]
        self.min_yrand = rand_area[2]
        self.max_yrand = rand_area[3]
        self.expand_dis = expand_dis
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.node_list = []
        self.d = 0.33
        self.dt = 0.1
        self.max_vel = 4
        self.vel_steps = 4
        self.steer_configs = [0, 0.1, 0.2, 0.3]
        self.vel_configs = [1, 2, 3, 4]

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
        if end.z != 0:
            #print("z != 0, point not valid for car")
            return None

        self.node_list = [start]
        for i in range(self.max_iter):
            rnd = self.get_random_point(end)
            nind = self.get_nearest_list_index(rnd)

            new_node = self.steer(rnd, nind)
            node_path = Seg(self.node_list[nind], new_node)
            if self.check_collision(obstacle_list, node_path):
                self.node_list.append(new_node)

            if self.close_to_goal(end, new_node):
                if self.check_collision(obstacle_list, Seg(new_node, end)):
                    path = self.gen_final_course(start, end, new_node)
                    # path = path[::-1]
                    # for i in range(len(path)):
                    #   print(path[i])
                    return path[::-1] #self.path_smoothing(obstacle_list, path[::-1], 100)

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

        nnx = nearest_node.x
        nny = nearest_node.y
        nnyaw = nearest_node.yaw

        theta_rnd = math.atan2(rnd[1] - nny, rnd[0] - nnx)
        theta_diff = nnyaw - theta_rnd
        if abs(theta_diff) <= math.pi/2:
            vel = 1
            if theta_diff <= 0:
                dir = 1
            else:
                dir = -1
        else:
            vel = -1
            if theta_diff <= 0:
                dir = -1
            else:
                dir = 1

        tmp_cost = []
        tmp_node = []

        for steer in self.steer_configs:
            for speed in self.vel_configs:
                cmd_vel = speed*self.dt*vel
                if steer == 0:
                    x_next = cmd_vel*math.cos(nnyaw) + nnx
                    y_next = cmd_vel*math.sin(nnyaw) + nny
                    yaw_next = nnyaw
                else:
                    tan_theta = math.tan(dir*steer)
                    c = cmd_vel*tan_theta / self.d
                    x_next = self.d*(math.sin(c + nnyaw) - math.sin(nnyaw))/tan_theta + nnx
                    y_next = self.d*(math.cos(nnyaw) - math.cos(c + nnyaw))/tan_theta + nny
                    yaw_next = c + nnyaw
                    if yaw_next > math.pi:
                        yaw_next = yaw_next - 2*math.pi
                    elif yaw_next < -math.pi:
                        yaw_next = yaw_next + 2*math.pi
                tmp_cost.append((x_next - rnd[0]) ** 2 + (y_next - rnd[1]) ** 2)
                tmp_node.append(Node(x_next, y_next, 0, yaw_next))

        minind = tmp_cost.index(min(tmp_cost))
        new_node = tmp_node[minind]

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
                   0]
        else:  # goal point sampling
            rnd = [end.x, end.y, 0]

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
        if random.randint(0, 100) < 5:
            return random.randint(0, len(self.node_list)-1)
        else:
            dlist = []
            for node in self.node_list:
                node_theta = math.atan2(rnd[1] - node.y, rnd[0] - node.x)
                if node_theta < 0:
                    node_theta = node_theta + math.pi
                tmp_yaw = node.yaw
                if tmp_yaw < 0:
                    tmp_yaw = tmp_yaw + math.pi
                dlist.append((node.x - rnd[0]) ** 2 + (node.y - rnd[1]) ** 2 + (tmp_yaw - node_theta) ** 2)
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
        path = [Pos(np.array([end.x, end.y, 0]))]

        node = last_node
        while node.parent is not None:
            path.append(Pos(np.array([node.x, node.y, 0, node.yaw])))
            node = node.parent
        path.append(Pos(np.array([start.x, start.y, 0, start.yaw])))
        return path

    def close_to_goal(self, end: Node, node: Node) -> bool:
        dist = math.sqrt((end.x - node.x)**2 + (end.y - node.y)**2)
        if dist <= self.expand_dis:
            if dist <= 0.2:
                return True

            end_theta = math.atan2(end.y - node.y, end.x - node.x)
            if end_theta < 0:
                end_theta = end_theta + math.pi
            tmp_yaw = node.yaw
            if tmp_yaw < 0:
                tmp_yaw = tmp_yaw + math.pi
            theta_diff = tmp_yaw - end_theta

            if abs(theta_diff) <= 0.2:
                return True
            else:
                return False
        else:
            return False

    def path_smoothing(self, obstacle_list: list, path: list, max_iter: int) -> list:
        """
        smooth path
        :param obstacle_list:
        :param path:
        :param max_iter:
        :return:
        """

        for _ in range(max_iter):
            path_len = len(path)
            pickPoints = [random.randint(0, path_len-1), random.randint(0, path_len-1)]
            if not ((pickPoints[0] == pickPoints[1]) or (abs(pickPoints[0] - pickPoints[1]) == 1)):
                # don't waste cpu time if points are the same or sequential
                pickPoints.sort()

                point0 = path[pickPoints[0]]
                point1 = path[pickPoints[1]]
                if pickPoints[1] == (path_len-1):
                    end_theta = math.atan2(point1.y - point0.y, point1.x - point0.x)
                    theta_diff = point0.yaw - end_theta
                else:
                    theta_diff = point0.yaw - point1.yaw

                if abs(theta_diff) <= 0.3:
                    pass
                else:
                    continue
                sub_seg = Seg(point0, point1)
                if not self.check_collision(obstacle_list, sub_seg):
                    continue
                path = path[0:pickPoints[0]+1] + path[pickPoints[1]:]
        return path

#
# a = RRT()
# p1 = Pos(np.array([-2, 0, 0]))
# p2 = Pos(np.array([-2, 0.5, 0]))
#
# from src.motion.cylobs import CylObs
# o1 = CylObs(Pos(np.array([0, 0, 0])), 0.5)
#
# import time
# loops = 1
# start_time = time.time()
# for i in range(loops):
#     p = a.find_path(p1, p2, [o1])
# elapsed_time = time.time() - start_time
# # print(elapsed_time/loops)
# # print(p)
# # for i in range(len(p)):
# #     print(p[i])

