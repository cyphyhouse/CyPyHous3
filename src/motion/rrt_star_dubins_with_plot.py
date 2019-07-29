"""
Path Planning Sample Code with RRT and Dubins path

author: AtsushiSakai(@Atsushi_twi)

"""

import copy
import math
import random

import dubins_path_planning
import matplotlib.pyplot as plt
import numpy as np

from src.motion.pos import Obs

show_animation = True


class RRT_DUBINS():
    """
    Class for RRT Planning
    """

    def __init__(self, rand_area: list = None, expand_dis: float = 0.1, goal_sample_rate: int = 15,
                 max_iter: int = 100):
        super(RRT_DUBINS, self).__init__()
        if rand_area is None:
            rand_area = [-12.5, 12.5]
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.expand_dis = expand_dis
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter

    def find_path(self, start_point, end_point, obstacle_list, animation=True):
        """
        Pathplanning

        animation: flag for animation on or off
        """

        node_list = [start_point]
        for i in range(self.max_iter):
            rnd = self.get_random_point(end_point)
            nind = self.GetNearestListIndex(node_list, rnd)

            newNode = self.steer(rnd, nind, node_list)
            #  print(newNode.cost)

            if self.CollisionCheck(newNode, obstacle_list):
                nearinds = self.find_near_nodes(newNode, node_list)
                newNode = self.choose_parent(newNode, nearinds, node_list, obstacle_list)
                node_list.append(newNode)
                self.rewire(newNode, nearinds, node_list, obstacle_list)

            if animation and i % 5 == 0:
                self.DrawGraph(start_point, end_point, node_list, obstacle_list, rnd)

        # generate course
        lastIndex = self.get_best_last_index(end_point, node_list)
        #  print(lastIndex)

        if lastIndex is None:
            return None

        path = self.gen_final_course(start_point, end_point, lastIndex, node_list)
        return path

    def choose_parent(self, newNode, nearinds, node_list, obstacle_list):
        if not nearinds:
            return newNode

        dlist = []
        for i in nearinds:
            tNode = self.steer(newNode, i, node_list)
            if self.CollisionCheck(tNode, obstacle_list):
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

    def pi_2_pi(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def steer(self, rnd, nind, node_list):
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

    def get_random_point(self, end_point):

        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = [random.uniform(self.min_rand, self.max_rand),
                   random.uniform(self.min_rand, self.max_rand),
                   random.uniform(-math.pi, math.pi)
                   ]
        else:  # goal point sampling
            rnd = [end_point.x, end_point.y, end_point.yaw]

        node = Node(rnd[0], rnd[1], rnd[2])

        return node

    def get_best_last_index(self, end_point, node_list):
        #  print("get_best_last_index")

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

    def gen_final_course(self, start_point, end_point, goalind, node_list):
        path = [[end_point.x, end_point.y]]
        while node_list[goalind].parent is not None:
            node = node_list[goalind]
            for (ix, iy) in zip(reversed(node.path_x), reversed(node.path_y)):
                path.append([ix, iy])
            #  path.append([node.x, node.y])
            goalind = node.parent
        path.append([start_point.x, start_point.y])
        return path

    def calc_dist_to_goal(self, x, y, end_point):
        return np.linalg.norm([x - end_point.x, y - end_point.y])

    def find_near_nodes(self, newNode, node_list):
        nnode = len(node_list)
        r = 50.0 * math.sqrt((math.log(nnode) / nnode))
        #  r = self.expandDis * 5.0
        dlist = [(node.x - newNode.x) ** 2 +
                 (node.y - newNode.y) ** 2 +
                 (node.yaw - newNode.yaw) ** 2
                 for node in node_list]
        nearinds = [dlist.index(i) for i in dlist if i <= r ** 2]
        return nearinds

    def rewire(self, newNode, nearinds, node_list, obstacle_list):

        nnode = len(node_list)

        for i in nearinds:
            nearNode = node_list[i]
            tNode = self.steer(nearNode, nnode - 1, node_list)

            obstacleOK = self.CollisionCheck(tNode, obstacle_list)
            imporveCost = nearNode.cost > tNode.cost

            if obstacleOK and imporveCost:
                #  print("rewire")
                node_list[i] = tNode

    def DrawGraph(self, start_point, end_point, node_list, obstacle_list: list, rnd=None):  # pragma: no cover
        """
        Draw Graph
        """
        plt.clf()
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
        for node in node_list:
            if node.parent is not None:
                plt.plot(node.path_x, node.path_y, "-g")
                #  plt.plot([node.x, self.nodeList[node.parent].x], [
                #  node.y, self.nodeList[node.parent].y], "-g")

        for obs in obstacle_list:
            plt.plot(obs.x, obs.y, "ok", ms=30 * obs.radius)

        dubins_path_planning.plot_arrow(
            start_point.x, start_point.y, start_point.yaw)
        dubins_path_planning.plot_arrow(
            end_point.x, end_point.y, end_point.yaw)

        plt.axis([-2, 15, -2, 15])
        plt.grid(True)
        plt.pause(0.01)

        #  plt.show()
        #  input()

    def GetNearestListIndex(self, node_list, rnd):
        dlist = [(node.x - rnd.x) ** 2 +
                 (node.y - rnd.y) ** 2 +
                 (node.yaw - rnd.yaw) ** 2 for node in node_list]
        minind = dlist.index(min(dlist))

        return minind

    def CollisionCheck(self, node, obstacle_list):

        for obstacle in obstacle_list:
            for (ix, iy) in zip(node.path_x, node.path_y):
                dx = obstacle.x - ix
                dy = obstacle.y - iy
                d = dx * dx + dy * dy
                if d <= obstacle.radius ** 2:
                    return False  # collision

        return True  # safe


class Node():
    """
    RRT Node
    """

    def __init__(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.path_x = []
        self.path_y = []
        self.path_yaw = []
        self.cost = 0.0
        self.parent = None


def main():
    print("Start rrt star with dubins planning")

    # ====Search Path with RRT====
    obstacle_list = [
        Obs(5, 5, 1),
        Obs(3, 6, 2),
        Obs(3, 8, 2),
        Obs(3, 10, 2),
        Obs(7, 5, 2),
        Obs(9, 5, 2)
    ]  # [x,y,size(radius)]

    # Set Initial parameters
    start = Node(0.0, 0.0, np.deg2rad(0.0))
    goal = Node(10.0, 10.0, np.deg2rad(0.0))

    rrt = RRT_DUBINS(rand_area=[-2.0, 15.0])
    path = rrt.find_path(start, goal, obstacle_list, animation=show_animation)

    # Draw final path
    if show_animation:  # pragma: no cover
        rrt.DrawGraph(start,goal,[],obstacle_list)
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
        plt.grid(True)
        plt.pause(0.001)

        plt.show()


if __name__ == '__main__':
    main()
