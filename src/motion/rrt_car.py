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
                 max_iter: int = 500):
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
        self.d = 0.3
        self.vel_steps = 2
        self.configs = [[expand_dis, 0], [expand_dis, 0.1], [expand_dis, 0.2], [expand_dis, 0.3],
                        [expand_dis, -0.1], [expand_dis, -0.2], [expand_dis, -0.3]]

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
            print("z != 0, point not valid for car")
            return None

        self.node_list = [start]
        for i in range(self.max_iter):
            rnd = self.get_random_point(end)
            nind = self.get_nearest_list_index(rnd)

            new_node = self.steer(rnd, nind)
            node_path = Seg(self.node_list[nind], new_node)
            if self.check_collision(obstacle_list, node_path):
                self.node_list.append(new_node)

            if self.calc_dist_to_goal(end, new_node.x, new_node.y) <= self.expand_dis/2:
                if self.check_collision(obstacle_list, Seg(new_node, end)):
                    path = self.gen_final_course(start, end, new_node)
                    # path = path[::-1]
                    # for i in range(len(path)):
                    #   print(path[i])
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
        
        theta_rnd = math.atan2(rnd[1] - nearest_node.y, rnd[0] - nearest_node.x)
        theta_diff = nearest_node.yaw - theta_rnd
        if abs(theta_diff) <= math.pi/2:
            vel = self.expand_dis
        else:
            vel = -self.expand_dis
            

        tmp_cost = []
        tmp_node = []

        for cmd in self.configs:
            for i in range(1, self.vel_steps+1):
                cmd_vel = (i/self.vel_steps)*vel
                if cmd[1] == 0:
                    x_next = cmd_vel*math.cos(nearest_node.yaw) + nearest_node.x
                    y_next = cmd_vel*math.sin(nearest_node.yaw) + nearest_node.y
                    yaw_next = nearest_node.yaw
                else:
                    tan_theta = math.tan(cmd[1])
                    c = cmd_vel*tan_theta / self.d
                    x_next = self.d*(math.cos(nearest_node.yaw) - math.cos(c + nearest_node.yaw))/tan_theta + nearest_node.x
                    y_next = self.d*(math.sin(c + nearest_node.yaw) - math.sin(nearest_node.yaw))/tan_theta + nearest_node.y
                    yaw_next = c + nearest_node.yaw
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

    def calc_dist_to_goal(self, end: Node, x: float, y: float) -> float:
        """
        calculate the distance to goal
        :param end:
        :param x:
        :param y:
        :return:
        """
        return np.linalg.norm([x - end.x, y - end.y, 0])
    
    def path_smoothing(self, obstacle_list: list, path: list, max_iter: int):
        """
        smooth path
        :param path:
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
                if (pickPoints[1] == path_len-1):
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

'''
a = RRT()
p1 = Pos(np.array([-2, 0, 0]))
p2 = Pos(np.array([-2, 0.5, 0]))

from src.motion.cylobs import CylObs
o1 = CylObs(Pos(np.array([0, 0, 0])), 0.5)

import time
loops = 1
start_time = time.time()
for i in range(loops):
    p = a.find_path(p1, p2)
elapsed_time = time.time() - start_time
print(elapsed_time/loops)
print(p)
for i in range(len(p)):
    print(p[i])
'''
