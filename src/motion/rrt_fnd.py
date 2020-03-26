"""
Path Planning Sample Code with RRT*

Author: AtsushiSakai(@Atsushi_twi)

Modifications for use in CyPhyHouse made by Amelia Gosse (gossea)
"""

# TODO: revisit documentation.
import copy
import math
import random
from typing import Union

import numpy as np

from src.motion.planner import Planner
from src.motion.pos_types import Pos, Node, to_node, Seg

import pygame


class RRT(Planner):
    """
    Class for RRT* Planning
    """

    ARENA_WIDTH = 10

    def __init__(self, rand_area: list = None, expand_dis: float = 0.25, goal_sample_rate: int = 15,
                 max_iter: int = 2000, animation=True, getPath=True):
        super(RRT, self).__init__()
        if rand_area is None:
            rand_area = [-RRT.ARENA_WIDTH, RRT.ARENA_WIDTH]
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.expand_dis = expand_dis
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.start = None
        self.end = None 
        self.node_list = {}
        self.obstacle_list = []
        self.nodeInd = 0
        self.getPath = getPath

        self.d = 0.33
        self.dt = 0.1
        self.max_vel = 3
        self.vel_steps = 3
        self.steer_configs = [0, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35]
        self.vel_configs = [1, 2, 3]
        self.steer_configs.reverse()
        self.vel_configs.reverse()

        self.animation = animation  
        if self.animation:
            XDIM = 1000
            YDIM = 1000
            windowSize = [XDIM, YDIM]
            self.screen = pygame.display.set_mode(windowSize)

    def find_path(self, start: Pos, end: Pos, obstacle_list: Union[list, None] = None,
                  search_until_max_iter: bool = False) -> \
            Union[list, None]:
        """15
        RRT* Path Planning
        search_until_max_iter: Search until max iteration for path improving or not
        """
        if obstacle_list is None:
            self.obstacle_list = []
        else:
            self.obstacle_list = obstacle_list
        print("here, start:", start, "end:", end)
        start = to_node(start)
        end = to_node(end)
        if end.z > 0.01 or end.z < -0.01:
            print("z != 0, point not valid for car")
            end.z = 0
            start.z = 0
            pass
        
        self.start = start 
        self.end = end
        self.nodeInd = 0
        self.node_list = {self.nodeInd: self.start}
        self.node_list[self.nodeInd].parent = None

        while self.nodeInd < self.max_iter:
            self.nodeInd += 1
            rnd = self.get_random_point(end) # with goal bias
            nind = self.get_nearest_list_index(rnd)
            new_node = self.steerD(rnd, nind)
            if new_node.x > RRT.ARENA_WIDTH or new_node.x < -RRT.ARENA_WIDTH or new_node.y > RRT.ARENA_WIDTH or new_node.y < -RRT.ARENA_WIDTH:
                    continue
            node_path = Seg(self.node_list[nind], new_node)
            if self.__collision_check(node_path):
                nearinds = self.find_near_nodes(new_node, 5)
                new_node = self.choose_parent(new_node, nearinds)
                self.node_list[self.nodeInd] = new_node
                if new_node.x > RRT.ARENA_WIDTH or new_node.x < -RRT.ARENA_WIDTH or new_node.y > RRT.ARENA_WIDTH or new_node.y < -RRT.ARENA_WIDTH:
                    continue
                self.rewire(self.nodeInd, new_node, nearinds)
                self.node_list[new_node.parent].children.add(self.nodeInd)

                # Force removal
                if len(self.node_list) > self.max_iter:
                    leaves = [key for key, node in self.node_list.items() if len(node.children) == 0 and len(self.node_list[node.parent].children) > 1]
                    if len(leaves) > 1:
                        ind = leaves[random.randint(0, len(leaves)-1)]
                        self.node_list[self.node_list[ind].parent].children.discard(ind)
                        self.node_list.pop(ind)
                    else:
                        leaves = [key for key, node in self.node_list.items() if len(node.children) == 0]
                        ind = leaves[random.randint(0, len(leaves)-1)]
                        self.node_list[self.node_list[ind].parent].children.discard(ind)
                        self.node_list.pop(ind)

                # generate course
                if self.getPath and (self.close_to_goal(new_node) or self.get_best_last_index()):
                    if self.__collision_check(Seg(new_node, self.end)):
                        path = self.gen_final_course(self.nodeInd)
                        #path = path[::2]
                        return path[::-1]

            if self.animation:
                # self.update_obstacles()
                self.DrawGraph()

        print("Reached max iteration")
        # self.node_list.clear()
        last_index = self.get_best_last_index()
        if self.getPath and last_index:
            path = self.gen_final_course(last_index)
            #path = path[::2]
            return path[::-1]

        return None

    def update_obstacles(self, obstacles=None, position=None, tooClose=True):
        """
        this function adds obstacle to the obstacles in 2 ways
        if input param is None, then use pygame to catch mouse input to add obstacle
        otherwise, add input obstacle to the obstacle list
        :param obstacle: new obstacle to add to list
        :return:
        """
        if not obstacles:
            for e in pygame.event.get():
                if e.type == pygame.MOUSEBUTTONDOWN:
                    if e.button == 1:
                        o1 = RectObs(Pos(np.array([(e.pos[0]/50)-10, (e.pos[1]/50)-10, 1])), np.array([0.5, 0.5, 0.5]))
                        self.obstacle_list.append(o1)
                        separate_tree = self.path_validation()
                        print("got separate tree", separate_tree)
                        self.tree_validation(separate_tree)
                        print("tree validation complete")
                        self.reconnect(separate_tree)
                        print("got reconnect")
                        self.regrow()
                        print("got regrow")
        else:
            print("planner receives the update obs signal")

            if self.obstacle_list != obstacles:
                self.obstacle_list = obstacles
                if self.animation:
                    self.DrawGraph()
                
                if not tooClose:
                    return None
                separate_tree = self.path_validation()
                print("create separate tree: ", separate_tree)
                if not separate_tree:
                    print("path not blocked")
                    last_index = self.get_best_last_index()
                    if self.getPath and last_index:
                        path = self.gen_final_course(last_index)

                        minind = np.argmin([(node-position).magnitude() for node in path])
                        path = path[:minind]
                        return path[::-1]
                print("prune the tree")
                self.tree_validation(separate_tree)
                print("regrow")
                path = self.regrow()
                if not path:
                    return None
                minind = np.argmin([(node - position).magnitude() for node in path])
                path = path[:minind]
                return path[::-1]

    def reconnect(self, separate_tree):
        """
        reconnect separate tree with parent tree if there are parent nodes within the connect range from separate tree
        :param separate_tree:
        :return: nothing
        """
        for node in separate_tree.values():
            for nodeInd in self.node_list:
                if np.linalg.norm([node.x - self.node_list[nodeInd].x, node.y - self.node_list[nodeInd].y]) > 0.1:
                    continue
                nodePath = Seg(node, self.node_list[nodeInd])
                if self.__collision_check(nodePath):
                    node.parent = nodeInd
                    return nodeInd
        return None

    def regrow(self):
        while self.nodeInd < self.max_iter:
            self.nodeInd += 1
            print('in regrow', self.nodeInd)
            rnd = self.get_random_point(self.end)  # with goal bias
            nind = self.get_nearest_list_index(rnd)
            new_node = self.steerD(rnd, nind)
            if new_node.x > RRT.ARENA_WIDTH or new_node.x < -RRT.ARENA_WIDTH or new_node.y > RRT.ARENA_WIDTH or new_node.y < -RRT.ARENA_WIDTH:
                    continue
            node_path = Seg(self.node_list[nind], new_node)
            if self.__collision_check(node_path):
                nearinds = self.find_near_nodes(new_node, 5)
                new_node = self.choose_parent(new_node, nearinds)
                if new_node.x > RRT.ARENA_WIDTH or new_node.x < -RRT.ARENA_WIDTH or new_node.y > RRT.ARENA_WIDTH or new_node.y < -RRT.ARENA_WIDTH:
                    continue
                self.node_list[self.nodeInd] = new_node
                self.rewire(self.nodeInd, new_node, nearinds)
                self.node_list[new_node.parent].children.add(self.nodeInd)

                # Force removal
                if len(self.node_list) > self.max_iter:
                    leaves = [key for key, node in self.node_list.items() if
                              len(node.children) == 0 and len(self.node_list[node.parent].children) > 1]
                    if len(leaves) > 1:
                        ind = leaves[random.randint(0, len(leaves) - 1)]
                        self.node_list[self.node_list[ind].parent].children.discard(ind)
                        self.node_list.pop(ind)
                    else:
                        leaves = [key for key, node in self.node_list.items() if len(node.children) == 0]
                        ind = leaves[random.randint(0, len(leaves) - 1)]
                        self.node_list[self.node_list[ind].parent].children.discard(ind)
                        self.node_list.pop(ind)

                last_index = self.get_best_last_index()
                print(last_index)
                if self.getPath and last_index:
                    path = self.gen_final_course(last_index)
                    # path = path[::2]
                    return path

            if self.animation:
                # self.update_obstacles()
                self.DrawGraph()

        print("Reached max iteration")
        # self.node_list.clear()
        last_index = self.get_best_last_index()
        print(last_index)
        if self.getPath and last_index:
            path = self.gen_final_course(last_index)
            # path = path[::2]
            print(path)
            return path[::-1]

    def path_validation(self):
        """
        validate the current path from goal towards start.
        if any part of the path is blocked by obstacles, remove it
        :return: nothing
        """
        lastIndex = self.get_best_last_index()
        separate_tree = {}
        if lastIndex is not None:
            while self.node_list[lastIndex].parent is not None:
                nodeInd = lastIndex
                lastIndex = self.node_list[lastIndex].parent
                separate_tree[nodeInd] = self.node_list[nodeInd]
                dx = self.node_list[nodeInd].x - self.node_list[lastIndex].x
                dy = self.node_list[nodeInd].y - self.node_list[lastIndex].y
                d = math.sqrt(dx ** 2 + dy ** 2)
                theta = math.atan2(dy, dx)
                if not self.check_collision_extend(self.node_list[lastIndex], theta, d):
                    break
                if lastIndex == 0:
                    return {}
        return separate_tree

    def tree_validation(self, separate_tree):
        """
        validate the entire search tree whether any branches has been collided with added obstacles
        :return: nothing
        """
        for nodeInd in list(self.node_list):
            if nodeInd not in self.node_list:
                continue
            if nodeInd in separate_tree:
                continue
            if nodeInd == 0:
                continue
            try: 
                nodePath = Seg(self.node_list[nodeInd], self.node_list[self.node_list[nodeInd].parent])
            except KeyError:
                continue
            if not self.__collision_check(nodePath):
                self.node_list[self.node_list[nodeInd].parent].children.discard(nodeInd)
                self.remove_branch(nodeInd)

    def remove_branch(self, nodeInd):
        """
        remove branch that has become obsolete that is connected nodeInd
        :param nodeInd:
        :return: nothing
        """
        for ix in self.node_list[nodeInd].children:
            self.remove_branch(ix)
        self.node_list.pop(nodeInd)

    def choose_parent(self, new_node: Node, nearinds: list) -> Node:
        """
        choose the parent for each node.
        :param new_node:
        :param nearinds:
        :return:
        """

        if len(nearinds) == 0:
            return new_node

        dlist = []
        for i in nearinds:
            dx = new_node.x - self.node_list[i].x
            dy = new_node.y - self.node_list[i].y
            d = math.sqrt(dx ** 2 + dy ** 2)
            theta = math.atan2(dy, dx)

            if self.check_collision_extend(self.node_list[i], theta, d):
                dlist.append(self.node_list[i].cost + d)
            else:
                dlist.append(float("inf"))

        mincost = min(dlist)
        minind = nearinds[dlist.index(mincost)]

        if mincost == float("inf"):
            print("mincost is inf")
            return new_node  # NOTE

        new_node.cost = mincost
        new_node.parent = minind

        return new_node

    def steer(self, rnd: list, nind: int) -> Node:
        """
        steer vehicle.
        :param node_list:
        :param rnd:
        :param nind:
        :return:
        """

        # expand tree
        nearest_node = self.node_list[nind]
        theta = math.atan2(rnd[1] - nearest_node.y, rnd[0] - nearest_node.x)
        new_node = Node(rnd[0], rnd[1], 0)
        current_distance = math.sqrt(
            (rnd[1] - nearest_node.y) ** 2 + (rnd[0] - nearest_node.x) ** 2)
        # Find a point within expand_dis of nind, and closest to rnd
        if current_distance <= self.expand_dis:
            pass
        else:
            new_node.x = nearest_node.x + self.expand_dis * math.cos(theta)
            new_node.y = nearest_node.y + self.expand_dis * math.sin(theta)
        new_node.cost = nearest_node.cost + self.expand_dis
        new_node.parent = nind
        return new_node

    def steerD(self, rnd: list, nind: int) -> Node:
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

                if (abs(x_next) <= self.max_rand) or (abs(y_next) <= self.max_rand):
                    tmp_cost.append((x_next - rnd[0]) ** 2 + (y_next - rnd[1]) ** 2)
                    tmp_node.append(Node(x_next, y_next, 0, yaw_next))

        if tmp_cost != []:
            minind = tmp_cost.index(min(tmp_cost))
            new_node = tmp_node[minind]
            new_node.parent = nind
        else:
            new_node = None
        return new_node

    def get_random_point(self, node: Pos) -> list:
        """
        function to get a random point near the end
        :param node:
        :return:
        """

        node_path = Seg(self.start, self.end)
        sample_rate = self.goal_sample_rate
        if self.__collision_check(node_path):
            sample_rate = 60
        if random.randint(0, 100) > sample_rate:
            rnd = [random.uniform(self.min_rand, self.max_rand),
                   random.uniform(self.min_rand, self.max_rand), 0]
        else:  # goal point sampling
            rnd = [node.x, node.y, 0]

        return rnd

    def get_best_last_index(self) -> Union[int, None]:
        """
        function to get best last index.
        :param node_list:
        :param end:
        :return:
        """

        disglist = [(key, self.calc_dist_to_goal(node.x, node.y)) for key, node in self.node_list.items()]
        goalinds = [key for key, distance in disglist if distance <= self.expand_dis*2]

        if len(goalinds) == 0:
            return None

        mincost = min([self.node_list[key].cost for key in goalinds])
        for i in goalinds:
            if self.node_list[i].cost == mincost:
                return i

        return None

    def rewire(self, newNodeInd: int, new_node: Node, nearinds: list) -> None:
        """
        rewiring function.
        :param node_list:
        :param obstacle_list:
        :param new_node:
        :param nearinds:
        :return:
        """
        nnode = len(self.node_list)
        for i in nearinds:
            near_node = self.node_list[i]

            dx = new_node.x - near_node.x
            dy = new_node.y - near_node.y
            d = math.sqrt(dx ** 2 + dy ** 2)

            scost = new_node.cost + d

            if near_node.cost > scost:
                theta = math.atan2(dy, dx)
                if self.check_collision_extend(near_node, theta, d):
                    self.node_list[near_node.parent].children.discard(i)
                    near_node.parent = newNodeInd
                    near_node.cost = scost
                    new_node.children.add(i)

    def check_collision_extend(self, near_node: Node, theta: float, d: float):
        """
        extended check collision
        :param near_node:
        :param theta:
        :param d:
        :return:
        """

        tmp_node = Node(near_node.x, near_node.y)

        for i in range(int(d / self.expand_dis)):
            tmp_node.x += self.expand_dis * math.cos(theta)
            tmp_node.y += self.expand_dis * math.sin(theta)
            node_path = Seg(near_node, tmp_node)
            if not self.__collision_check(node_path):
                return False
        return True

    def __collision_check(self, dir_seg: Seg) -> bool:
        """
        check collision callback
        :param dir_seg:
        :return:
        """

        for obs in self.obstacle_list:
            if not obs.collision_check(dir_seg) or not obs.collision_check(dir_seg.start) or not obs.collision_check(dir_seg.end):
                return False
        return True

    def get_nearest_list_index(self, rnd: list) -> int:
        """
        function for returning the list index of the nearest point
        :param node_list:
        :param rnd:
        :return:
        """
        dlist = np.subtract(np.array([(node.x, node.y) for node in self.node_list.values()]), (rnd[0],rnd[1]))**2
        dlist = np.sum(dlist, axis=1)
        minind = list(self.node_list.keys())[np.argmin(dlist)]
        return minind

    def find_near_nodes(self, new_node: Node, value: int) -> list:
        """
        TODO: add documentation for find_near_nodes
        :param node_list:
        :param new_node:
        :return:
        """
        r = self.expand_dis * value
        dlist = np.subtract(np.array([(node.x, node.y) for node in self.node_list.values()]), (new_node.x, new_node.y))**2
        dlist = np.sum(dlist, axis=1)
        nearinds = np.where(dlist <= r ** 2)
        nearinds = np.array(list(self.node_list.keys()))[nearinds]
        return nearinds

    def gen_final_course(self, goal_ind: int) -> list:
        """
        generate the final path
        :param node_list:
        :param start:
        :param end:
        :param goal_ind:
        :return:
        """
        path = [Pos(np.array([self.end.x, self.end.y, 0]))]
        node = self.node_list[goal_ind]
        seg_last_two_points = Seg(Pos(np.array([node.x, node.y, 0])), Pos(np.array([self.end.x, self.end.y, 0])))
        if (seg_last_two_points.length() <= 0.1):  # magic number (change to something better later)
            goal_ind = node.parent  # skip last point because it is very close to end point
        while self.node_list[goal_ind].parent is not None:
            node = self.node_list[goal_ind]
            path.append(Pos(np.array([node.x, node.y, 0])))
            goal_ind = node.parent
            if goal_ind == 0:
                break
        path.append(Pos(np.array([self.start.x, self.start.y, 0])))
        return path

    def calc_dist_to_goal(self, x: float, y: float) -> float:
        """
        calculate the distance to goal
        :param end:
        :param x:
        :param y:
        :return:
        """
        return np.linalg.norm([x - self.end.x, y - self.end.y])

    def close_to_goal(self, node: Node) -> bool:
        dist = math.sqrt((self.end.x - node.x) ** 2 + (self.end.y - node.y) ** 2)
        if dist <= self.expand_dis:
            if dist <= 0.2:
                return True

            end_theta = math.atan2(self.end.y - node.y, self.end.x - node.x)
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

    def DrawGraph(self, drawPath=True, separate_tree=None):
        self.screen.fill((255, 255, 255))
        for node in self.node_list.values():
            if node.parent is not None:
                # print(node.parent)
                pygame.draw.line(self.screen, (0,255,0), [int((self.node_list[node.parent].y+10)*50), int((self.node_list[node.parent].x+10)*50)], [int((node.y+10)*50), int((node.x+10)*50)])
        
        for node in self.node_list.values():
            if len(node.children) == 0:
                pygame.draw.circle(self.screen, (255, 0, 255), [int((node.y+10)*50), int((node.x+10)*50)], 2)

        for ob in self.obstacle_list:
            pygame.draw.rect(self.screen,(0,0,0), [((ob.position.y-ob.size[0]/2+10)*50, (ob.position.x-ob.size[1]/2+10)*50),(ob.size[0]*50, ob.size[1]*50)])

        if separate_tree:
            for idx in separate_tree:
                pygame.draw.circle(self.screen, (255, 204, 0),
                                   [int((separate_tree[idx].y + 10) * 50), int((separate_tree[idx].x + 10) * 50)], 5)

        pygame.draw.circle(self.screen, (255,0,0), [int((self.start.y+10)*50), int((self.start.x+10)*50)], 10)
        pygame.draw.circle(self.screen, (0,0,255), [int((self.end.y+10)*50), int((self.end.x+10)*50)], 10)

        if drawPath:
            lastIndex = self.get_best_last_index()
            if lastIndex is not None:
                path = self.gen_final_course(lastIndex)
                ind = len(path)
                while ind > 1:
                    pygame.draw.line(self.screen, (255, 0, 0), [int((path[ind-2].y+10)*50), int((path[ind-2].x+10)*50)], [int((path[ind-1].y+10)*50),int((path[ind-1].x+10)*50)])
                    ind-=1
            
        pygame.display.update()

if __name__ == "__main__":
    pygame.init()
    fpsClock = pygame.time.Clock()
    pygame.display.set_caption('Performing RRT')

    planner = RRT(animation=True, getPath=False)
    p1 = Pos(np.array([random.randint(1, 9), random.randint(1, 9), 0]))
    p2 = Pos(np.array([random.randint(1, 9), random.randint(1, 9), 0]))
    
    from src.motion.rectobs import RectObs
    o1 = RectObs(Pos(np.array([0, 0, 0])), np.array([1, 10, 1]))
    obstacle_list = [o1]
    p = planner.find_path(p1, p2, obstacle_list)
    print("Path found")

