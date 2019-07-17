"""
Path Planning Sample Code with RRT*

Author: AtsushiSakai(@Atsushi_twi)

Modifications for use in CyPhyHouse made by Amelia Gosse (gossea)
"""

import copy
import math
import random

import numpy as np


class Node():

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cost = 0.0
        self.parent = None


class RRT(object):
    """
    Class for RRT* Planning
    """

    def __init__(self):
        pass


    def plan(self,start: list, goal: list, obstacleList: list = [], randArea: list = [-5, 5],
             expandDis: float = 0.55555, goalSampleRate: int = 200, maxIter: int = 500):

        """
        Setting Parameters

        start: Start position [x,y]
        goal: Goal position [x,y]
        obstacleList: Obstacle positions [[x,y,size],...]
        randArea: Random sampling bounds [min,max]

        """
        # self.__motionautomaton = motionautomaton
        self.start = Node(start[0], start[1])
        self.end = Node(goal[0], goal[1])
        self.minrand = randArea[0]
        self.maxrand = randArea[1]
        self.expandDis = expandDis
        self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter
        self.obstacleList = obstacleList

    def Planning(self, search_until_maxiter: bool = False) -> list:
        """
        RRT* Path Planning
        search_until_maxiter: Search until max iteration for path improving or not
        """
        self.nodeList = [self.start]
        for i in range(self.maxIter):
            rnd = self.get_random_point()
            nind = self.GetNearestListIndex(self.nodeList, rnd)

            new_node = self.steer(rnd, nind)

            if self.__CollisionCheck(new_node, self.obstacleList):
                nearinds = self.find_near_nodes(new_node)
                new_node = self.choose_parent(new_node, nearinds)
                self.nodeList.append(new_node)
                self.rewire(new_node, nearinds)

            # generate course
            if not search_until_maxiter:
                lastIndex = self.get_best_last_index()
                if lastIndex:
                    path = self.gen_final_course(lastIndex)
                    return to_path(path[::-1])

        print("Reached max iteration")

        lastIndex = self.get_best_last_index()
        if lastIndex:
            path = self.gen_final_course(lastIndex)
            return to_path(path[::-1])

        return None

    def choose_parent(self, new_node, nearinds: list):
        if not nearinds:
            return new_node

        dlist = []
        for i in nearinds:
            dx = new_node.x - self.nodeList[i].x
            dy = new_node.y - self.nodeList[i].y
            d = math.sqrt(dx ** 2 + dy ** 2)
            theta = math.atan2(dy, dx)
            if self.check_collision_extend(self.nodeList[i], theta, d):
                dlist.append(self.nodeList[i].cost + d)
            else:
                dlist.append(float("inf"))

        mincost = min(dlist)
        minind = nearinds[dlist.index(mincost)]

        if mincost == float("inf"):
            print("mincost is inf")
            return new_node

        new_node.cost = mincost
        new_node.parent = minind

        return new_node

    def steer(self, rnd: list, nind: int):

        # expand tree
        nearest_node = self.nodeList[nind]
        theta = math.atan2(rnd[1] - nearest_node.y, rnd[0] - nearest_node.x)
        new_node = Node(rnd[0], rnd[1])
        currentDistance = math.sqrt(
            (rnd[1] - nearest_node.y) ** 2 + (rnd[0] - nearest_node.x) ** 2)
        # Find a point within expandDis of nind, and closest to rnd
        if currentDistance <= self.expandDis:
            pass
        else:
            new_node.x = nearest_node.x + self.expandDis * math.cos(theta)
            new_node.y = nearest_node.y + self.expandDis * math.sin(theta)
        new_node.cost = float("inf")
        new_node.parent = None
        return new_node

    def get_random_point(self):

        if random.randint(0, 100) > self.goalSampleRate:
            rnd = [random.uniform(self.minrand, self.maxrand),
                   random.uniform(self.minrand, self.maxrand)]
        else:  # goal point sampling
            rnd = [self.end.x, self.end.y]

        return rnd

    def get_best_last_index(self):

        disglist = [self.calc_dist_to_goal(
            node.x, node.y) for node in self.nodeList]
        goalinds = [disglist.index(i) for i in disglist if i <= self.expandDis]

        if not goalinds:
            return None

        mincost = min([self.nodeList[i].cost for i in goalinds])
        for i in goalinds:
            if self.nodeList[i].cost == mincost:
                return i

        return None

    def gen_final_course(self, goalind: int):
        path = [[self.end.x, self.end.y]]
        while self.nodeList[goalind].parent is not None:
            node = self.nodeList[goalind]
            path.append([node.x, node.y])
            goalind = node.parent
        path.append([self.start.x, self.start.y])
        return path

    def calc_dist_to_goal(self, x: float, y: float):
        return np.linalg.norm([x - self.end.x, y - self.end.y])

    def find_near_nodes(self, new_node):
        nnode = len(self.nodeList)
        r = 50.0 * math.sqrt((math.log(nnode) / nnode))
        dlist = [(node.x - new_node.x) ** 2 +
                 (node.y - new_node.y) ** 2 for node in self.nodeList]
        nearinds = [dlist.index(i) for i in dlist if i <= r ** 2]
        return nearinds

    def rewire(self, new_node, nearinds: list):
        nnode = len(self.nodeList)
        for i in nearinds:
            nearNode = self.nodeList[i]

            dx = new_node.x - nearNode.x
            dy = new_node.y - nearNode.y
            d = math.sqrt(dx ** 2 + dy ** 2)

            scost = new_node.cost + d

            if nearNode.cost > scost:
                theta = math.atan2(dy, dx)
                if self.check_collision_extend(nearNode, theta, d):
                    nearNode.parent = nnode - 1
                    nearNode.cost = scost

    def check_collision_extend(self, nearNode, theta: float, d: float):
        tmpNode = copy.deepcopy(nearNode)

        for i in range(int(d / self.expandDis)):
            tmpNode.x += self.expandDis * math.cos(theta)
            tmpNode.y += self.expandDis * math.sin(theta)
            if not self.__CollisionCheck(tmpNode, self.obstacleList):
                return False
        return True


    def GetNearestListIndex(self, nodeList: list, rnd: list):
        dlist = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1]) ** 2 for node in nodeList]
        minind = dlist.index(min(dlist))

        return minind

    def __CollisionCheck(self, node, obstacleList: list):
        for (ox, oy, size) in obstacleList:
            dx = ox - node.x
            dy = oy - node.y
            d = dx * dx + dy * dy
            if d <= size ** 2:
                return False  # collision

        return True  # safe



















'''
a = RRT()
a.plan([0,0,0],[1,1,0])
c = a.Planning()
d = [vec(0.5,0.5,0)]
t = path_is_close(c,d,-1)
print(t)
'''