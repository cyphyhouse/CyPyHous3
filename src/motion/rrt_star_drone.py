"""
Path Planning Sample Code with RRT*

Author: AtsushiSakai(@Atsushi_twi)

Modifications for use in CyPhyHouse made by Joao
"""

# TODO: revisit documentation.
import math
import random
from typing import Optional, Sequence, MutableSequence

import numpy as np

from src.motion.obstacle import Obstacle
from src.motion.planner import Planner
from src.motion.pos_types import Pos, Node, to_node, Seg, distance


def _collision_check(pos: Pos, obstacle_list: Sequence[Obstacle]) -> bool:
    """
    check collision callback
    :param pos:
    :param obstacle_list:
    :return:
    """
    return all(obs.isdisjoint(pos) for obs in obstacle_list)


def check_collision_extend(obstacle_list: Sequence[Obstacle], dir_seg: Seg) -> bool:
    """
    extended check collision
    :param obstacle_list:
    :param dir_seg:
    :return:
    """
    return all(obs.isdisjoint(dir_seg) for obs in obstacle_list)


def node_selection(node_list: Sequence[Node], target_pos: Pos, obstacle_list: Sequence[Obstacle]) \
        -> Optional[Node]:
    def dist_sq(n: Node) -> float:
        return (n.x - target_pos.x) ** 2 + (n.y - target_pos.y) ** 2 + (n.z - target_pos.z) ** 2

    for node in sorted(node_list, key=dist_sq):  # Start from nearest
        # Feasibility condition
        if check_collision_extend(obstacle_list, Seg(node, target_pos)):
            return node
    return None


def get_best_last_node(node_list: Sequence[Node], end: Pos, tolerance: float = 0.1) -> Optional[Node]:
    """
    function to get best last index.
    :param node_list:
    :param end:
    :param tolerance:
    :return:
    """
    last_node_list = [node for node in node_list if distance(end, node) <= tolerance]
    if not last_node_list:
        return None

    return min(last_node_list, key=lambda node: node.cost)


class RRT(Planner):
    """
    Class for RRT* Planning with Spline for Drones
    """

    def __init__(self, rand_area: Sequence[float] = (), expand_dis: float = 0.5, goal_sample_rate: int = 15,
                 max_iter: int = 200):
        super(RRT, self).__init__()
        if not rand_area:
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

    def find_path(self, start: Pos, end: Pos, obstacle_list: Sequence[Obstacle] = (),
                  search_until_max_iter: bool = False, tolerance: float = 0.1) -> \
            Optional[Sequence[Pos]]:
        """
        RRT* Path Planning
        search_until_max_iter: Search until max iteration for path improving or not
        """
        if end.z == 0:
            print("z = 0, point not valid for drone")
            return None
        if not _collision_check(end, obstacle_list):
            print("End position %s is inside obstacles." % str(end))
            return None
        if np.any(start.mk_arr() < self._rand_area.mins) or \
                np.any(self._rand_area.maxes < start.mk_arr()):
            print("Start position %s is out of sampling range %s" % (str(start), str(self._rand_area)))
            return None
        if np.any(end.mk_arr() < self._rand_area.mins) or \
                np.any(self._rand_area.maxes < end.mk_arr()):
            print("End position %s is out of sampling range %s" % (str(start), str(self._rand_area)))
            return None
        assert _collision_check(start, obstacle_list)

        start_node = to_node(start)
        node_list = [start_node]
        for i in range(self.max_iter):
            target_pos = self.get_random_point(end, obstacle_list)  # Randomly pick a target position

            nearest_feasible_node = node_selection(node_list, target_pos, obstacle_list)
            if not nearest_feasible_node:
                continue  # Resample

            # At least one node is feasible
            new_node = self.node_expansion(node_list, target_pos, nearest_feasible_node, obstacle_list)
            assert new_node.parent is not None

            # generate course
            if not search_until_max_iter and distance(new_node, end) <= tolerance:
                return gen_final_path(node_list, start_node, new_node)  # Return first path found

        print("Reached max iteration")
        last_node = get_best_last_node(node_list, end, tolerance)
        if last_node:
            return gen_final_path(node_list, start_node, last_node)

        return None

    def node_expansion(self, node_list: MutableSequence[Node], target_pos: Pos, nearest_node: Node,
                       obstacle_list: Sequence[Obstacle]) -> Node:
        new_node = self.steer(nearest_node, target_pos)
        nearinds = self.find_near_nodes(node_list, new_node)
        # expand tree
        new_node = self.choose_parent(node_list, obstacle_list, new_node, nearinds)
        assert new_node.parent is not None
        node_list.append(new_node)
        self.rewire(node_list, obstacle_list, new_node, nearinds)
        return new_node

    def choose_parent(self, node_list: Sequence[Node], obstacle_list: Sequence[Obstacle],
                      new_node: Node, nearinds: Sequence[int]) -> Node:
        """
        choose the parent for each node.
        :param node_list:
        :param obstacle_list:
        :param new_node:
        :param nearinds:
        :return:
        """
        assert nearinds
        mincost = float("inf")
        minind = -1
        for ind in nearinds:
            d_seg = Seg(node_list[ind], new_node)
            if check_collision_extend(obstacle_list, d_seg):
                cost = node_list[ind].cost + d_seg.length()
                if cost < mincost:
                    mincost = cost
                    minind = ind
        assert np.isfinite(mincost) and 0 <= minind < len(node_list)

        new_node.cost = mincost
        new_node.parent = minind
        return new_node

    def steer(self, nearest_node: Node, target_pos: Pos) -> Node:
        """
        steer vehicle.
        :param nearest_node:
        :param target_pos:
        :return:
        """
        seg_nn_to_rnd = Seg(nearest_node, target_pos)
        nn_to_rnd_uvec = seg_nn_to_rnd.direction()
        new_node = to_node(target_pos)

        # Find a point within expand_dis of nearest_node, and closest to rnd
        if seg_nn_to_rnd.length() <= self.expand_dis:
            pass
        else:
            new_node.x = nearest_node.x + self.expand_dis * nn_to_rnd_uvec.x
            new_node.y = nearest_node.y + self.expand_dis * nn_to_rnd_uvec.y
            new_node.z = nearest_node.z + self.expand_dis * nn_to_rnd_uvec.z
        new_node.cost = float("inf")
        new_node.parent = None
        return new_node

    def get_random_point(self, end: Pos, obstacle_list: Sequence[Obstacle] = ()) -> Pos:
        """
        function to get a random point near the end, and not inside obstacles
        :param end:
        :param obstacle_list:
        :return:
        """
        rnd = Pos(np.array([end.x, end.y, end.z]))
        if random.randint(0, 100) > self.goal_sample_rate:
            for i in range(self.max_iter):
                rnd = [random.uniform(self.min_xrand, self.max_xrand),
                       random.uniform(self.min_yrand, self.max_yrand),
                       random.uniform(self.min_zrand, self.max_zrand)]
                rnd = Pos(np.array(rnd))
                if _collision_check(rnd, obstacle_list):
                    return rnd
        else:  # goal point sampling
            pass
        assert _collision_check(rnd, obstacle_list)
        return rnd

    def rewire(self, node_list: Sequence[Node], obstacle_list: Sequence[Obstacle],
               new_node: Node, nearinds: Sequence[int]) -> None:
        """
        rewire nearby nodes to set the new node as parent if the cost is reduced.
        :param node_list:
        :param obstacle_list:
        :param new_node:
        :param nearinds:
        :return:
        """
        nnode = len(node_list)
        for i in nearinds:
            near_node = node_list[i]

            d_seg = Seg(near_node, new_node)
            scost = new_node.cost + d_seg.length()

            if near_node.cost > scost:
                if check_collision_extend(obstacle_list, d_seg):
                    near_node.parent = nnode - 1
                    near_node.cost = scost

    def find_near_nodes(self, node_list: Sequence[Node], new_node: Node) -> Sequence[int]:
        """
        TODO: add documentation for find_near_nodes
        :param node_list:
        :param new_node:
        :return:
        """
        r = self.expand_dis
        nearinds = [
            idx for idx, node in enumerate(node_list) if
            distance(new_node, node) <= 2*r  # At least the nearest feasible node will be included
        ]
        assert nearinds
        return nearinds


def get_nearest_list_index(node_list: Sequence[Node], target_pos: Pos) -> int:
    """
    function for returning the list index of the nearest node
    :param node_list:
    :param target_pos:
    :return:
    """
    dlist = [(node.x - target_pos.x) ** 2 + (node.y - target_pos.y) ** 2 + (node.z - target_pos.z) ** 2
             for node in node_list]
    minind = dlist.index(min(dlist))
    return minind


def gen_final_path(node_list: Sequence[Node], start_node: Node, last: Node) -> Sequence[Pos]:
    """ generate the final path excluding start node
    :param node_list:
    :param start_node:
    :param last:
    :return:
    """
    path = []
    curr_node = last
    while curr_node.parent is not None:
        path.append(curr_node)
        curr_node = node_list[curr_node.parent]
    assert curr_node == start_node
    path.reverse()
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
