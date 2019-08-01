"""

Path Planning Sample Code with RRT for car like robot.

author: AtsushiSakai(@Atsushi_twi)

"""
import matplotlib.pyplot as plt
import numpy as np
import copy
import math
import random
import src.motion.reeds_shepp_path_planning as reeds_shepp_path_planning
from typing import Union
from src.motion.planner import Planner
from src.motion.pos_types import Pos, Node, to_node, pos3d


class ReedsShepp(Planner):
    """
    Class for RRT* Planning with Reeds Shepp dynamics
    """

    def __init__(self, rand_area: list = None,
                 goalSampleRate: int = 15, maxIter: int = 100, curvature: float = 1.0, step_size: float = 0.1):
        super(ReedsShepp, self).__init__()
        if rand_area is None:
            rand_area = [-2.5, 2.5]
        self.minrand = rand_area[0]
        self.maxrand = rand_area[1]
        self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter
        self.curvature = curvature
        self.step_size = step_size

    def find_path(self, start: Pos, end: Pos, obstacle_list: Union[list, None] = None) -> Union[list, None]:
        if obstacle_list is None:
            obstacle_list = []
        start = to_node(start)
        end = to_node(end)
        if end.z != 0:
            print("z != 0, point not valid for car")
            return None
        self.nodeList = [start]
        for i in range(self.maxIter):
            rnd = self.get_random_point(end)
            nind = self.GetNearestListIndex(self.nodeList, rnd)

            newNode = self.steer(rnd, nind)
            if newNode is None:
                continue

            if self.CollisionCheck(newNode, obstacle_list):
                nearinds = self.find_near_nodes(newNode)
                newNode = self.choose_parent(newNode, nearinds, obstacle_list)
                if newNode is None:
                    continue
                self.nodeList.append(newNode)
                self.rewire(nearinds, obstacle_list)

        # Generate course
        lastIndex = self.get_best_last_index(end)
        if lastIndex is None:
            return None
        path = self.gen_final_course(lastIndex, start, end)
        # Reverse path order so final destination is last entry
        return path[::-1]

    def choose_parent(self, newNode, nearinds, obstacle_list: list):
        if not nearinds:
            return newNode

        dlist = []
        for i in nearinds:
            tNode = self.steer(newNode, i)
            if tNode is None:
                continue

            if self.CollisionCheck(tNode, obstacle_list):
                dlist.append(tNode.cost)
            else:
                dlist.append(float("inf"))

        mincost = min(dlist)
        minind = nearinds[dlist.index(mincost)]

        if mincost == float("inf"):
            print("mincost is inf")
            return newNode

        newNode = self.steer(newNode, minind)

        return newNode

    def pi_2_pi(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def steer(self, rnd, nind):

        nearestNode = self.nodeList[nind]

        px, py, pyaw, mode, clen = reeds_shepp_path_planning.reeds_shepp_path_planning(
            nearestNode.x, nearestNode.y, nearestNode.yaw, rnd.x, rnd.y, rnd.yaw, self.curvature, self.step_size)

        if px is None:
            return None

        newNode = copy.deepcopy(nearestNode)
        newNode.x = px[-1]
        newNode.y = py[-1]
        newNode.yaw = pyaw[-1]

        newNode.path_x = px
        newNode.path_y = py
        newNode.path_yaw = pyaw
        newNode.cost += sum([abs(c) for c in clen])
        newNode.parent = nind

        return newNode

    def get_random_point(self, end: Node):

        if random.randint(0, 100) > self.goalSampleRate:
            rnd = [random.uniform(self.minrand, self.maxrand),
                   random.uniform(self.minrand, self.maxrand),
                   random.uniform(-math.pi, math.pi)
                   ]
        else:  # Goal point sampling
            rnd = [end.x, end.y, end.yaw]

        node = Node(rnd[0], rnd[1], rnd[2])

        return node

    def get_best_last_index(self, end: Node):
        #  print("get_best_last_index")
        YAWTH = np.deg2rad(3.0)
        XYTH = 0.5

        goalinds = []
        for (i, node) in enumerate(self.nodeList):
            if self.calc_dist_to_goal(node.x, node.y, end) <= XYTH:
                goalinds.append(i)
        #  print("OK XY TH num is")
        #  print(len(goalinds))

        # angle check
        fgoalinds = []
        for i in goalinds:
            if abs(self.nodeList[i].yaw - end.yaw) <= YAWTH:
                fgoalinds.append(i)
        #  print("OK YAW TH num is")
        #  print(len(fgoalinds))

        if not fgoalinds:
            return None

        mincost = min([self.nodeList[i].cost for i in fgoalinds])
        for i in fgoalinds:
            if self.nodeList[i].cost == mincost:
                return i

        return None

    def gen_final_course(self, goalind, start: Node, end: Node):
        path = [[end.x, end.y]]
        while self.nodeList[goalind].parent is not None:
            node = self.nodeList[goalind]
            for (ix, iy) in zip(reversed(node.path_x), reversed(node.path_y)):
                path.append([ix, iy])
            goalind = node.parent
        path.append([start.x, start.y])
        return path

    def calc_dist_to_goal(self, x, y, end: Node):
        return np.linalg.norm([x - end.x, y - end.y])

    def find_near_nodes(self, newNode):
        nnode = len(self.nodeList)
        r = 50.0 * math.sqrt((math.log(nnode) / nnode))
        #  r = self.expandDis * 5.0
        dlist = [(node.x - newNode.x) ** 2 +
                 (node.y - newNode.y) ** 2 +
                 (node.yaw - newNode.yaw) ** 2
                 for node in self.nodeList]
        nearinds = [dlist.index(i) for i in dlist if i <= r ** 2]
        return nearinds

    def rewire(self, nearinds, obstacle_list: list):

        nnode = len(self.nodeList)

        for i in nearinds:
            nearNode = self.nodeList[i]
            tNode = self.steer(nearNode, nnode - 1)
            if tNode is None:
                continue

            obstacleOK = self.CollisionCheck(tNode, obstacle_list)
            improveCost = nearNode.cost > tNode.cost

            if obstacleOK and improveCost:
                #  print("rewire")
                self.nodeList[i] = tNode

    def GetNearestListIndex(self, nodeList, rnd):
        dlist = [(node.x - rnd.x) ** 2 +
                 (node.y - rnd.y) ** 2 +
                 (node.yaw - rnd.yaw) ** 2 for node in nodeList]
        minind = dlist.index(min(dlist))

        return minind

    def CollisionCheck(self, node, obstacle_list: list):

        for obs in obstacle_list:
            try:
                dx = obs.x - node.x
                dy = obs.y - node.y
                d = dx * dx + dy * dy
                if d <= obs.radius ** 2:
                    return False  # collision
            except AttributeError:
                print("obstacle might not be correctly formatted")

        return True  # safe


test = ReedsShepp()
path = test.find_path(pos3d(0., 0., 0.), pos3d(1., 2., 0.))
print(path)
