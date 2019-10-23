from src.config.configs import AgentConfig, MoatConfig
from src.harness.agentThread import AgentThread
from src.motion.pos_types import pos3d, Pos  # TODO Choose only one of them
from src.motion.cylobs import CylObs

from bisect import bisect
from collections import deque
import csv
import queue
from typing import List, Tuple
import rospy

import numpy as np
import matplotlib.pyplot as plt


LIDAR_RANGE = 2.5
LIDAR_ANGLE_MIN, LIDAR_ANGLE_MAX = -np.pi, np.pi

Grid = Tuple[int, int]


def _normalize_radians(rad: float) -> float:
    return (rad + np.pi) % (2 * np.pi) - np.pi


class GridMap:
    UNKNOWN = -1
    EMPTY = 0
    OCCUPIED = 1

    HALF_GRID_WIDTH = 11
    GRID_WIDTH = 2 * HALF_GRID_WIDTH

    def __init__(self, shape=(GRID_WIDTH, GRID_WIDTH)):
        """ Map should initialized with -1"""
        self.__grid_map = np.full(shape=shape, fill_value=GridMap.UNKNOWN, dtype=np.int8)
        for i in range(-GridMap.HALF_GRID_WIDTH, GridMap.HALF_GRID_WIDTH):
            self.__grid_map[i][-GridMap.HALF_GRID_WIDTH] = GridMap.OCCUPIED
            self.__grid_map[i][GridMap.HALF_GRID_WIDTH-1] = GridMap.OCCUPIED

            self.__grid_map[-GridMap.HALF_GRID_WIDTH][i] = GridMap.OCCUPIED
            self.__grid_map[GridMap.HALF_GRID_WIDTH-1][i] = GridMap.OCCUPIED

    def __add__(self, other):
        """ Merge maps by choosing the larger value """
        assert self.__grid_map.shape == other.__grid_map.shape \
            and self.__grid_map.dtype == other.__grid_map.dtype
        ret = GridMap(self.__grid_map.shape)
        np.maximum(self.__grid_map, other.__grid_map, out=ret.__grid_map)
        return ret

    def __getitem__(self, key):
        return self.__grid_map.__getitem__(key)

    def __setitem__(self, key, value):
        if key[0] >= GridMap.HALF_GRID_WIDTH or key[1] >= GridMap.HALF_GRID_WIDTH or key[0] < -GridMap.HALF_GRID_WIDTH or key[1] < -GridMap.HALF_GRID_WIDTH:
            return 
        return self.__grid_map.__setitem__(key, value)

    def __delitem__(self, key):
        return self.__grid_map.__delitem__(key)

    def __repr__(self):
        return repr(self.__grid_map.tolist())

    def show(self):
        viz_map = np.full(shape=self.__grid_map.shape, fill_value=-1, dtype=np.int8)
        for x in range(GridMap.GRID_WIDTH):
            for y in range(GridMap.GRID_WIDTH):
                _x, _y = x - GridMap.HALF_GRID_WIDTH, y - GridMap.HALF_GRID_WIDTH
                viz_map[y][x] = self.__grid_map[_x][_y]  # XXX the image has to be transposed

        plt.imshow(viz_map, origin='lower')
        plt.pause(0.5)

    @staticmethod
    def quantize(x: float, y: float) -> Grid:
        return tuple(np.floor([x, y]).astype(int))

    @staticmethod
    def _nbr_grids(grid: Grid) -> List[Grid]:
        x, y = grid
        return [(x-1, y), (x+1, y), (x, y-1), (x, y+1)]

    def get_frontier_grids(self, curr_grid: Grid) -> List[Grid]:
        """ Get the frontier empty grids that are next to unknown grids.
        These frontier nodes must be reachable from current grid thru a connected empty grids.
        :param curr_grid: Current grid
        :return: Frontier grids
        """
        assert self[curr_grid] != GridMap.OCCUPIED

        frontier = []
        visited = {curr_grid}
        q = deque()
        q.append(curr_grid)
        # Simple BFS to find the list of frontier grids. TODO optimization?
        while q:
            s = q.popleft()

            nbr_grids = self._nbr_grids(s)
            # If any of the neighbors is unknown, this grid is a frontier grid
            if any([self[n] == GridMap.UNKNOWN for n in nbr_grids]):
                frontier.append(s)
            # Add unvisited empty neighbors to queue
            q.extend([n for n in nbr_grids if n not in visited and self[n] == GridMap.EMPTY])

            visited |= set(nbr_grids)  # Add neighbors to visited

        return frontier


class BasicFollowApp(AgentThread):

    def __init__(self, agent_config: AgentConfig, moat_config: MoatConfig):
        super(BasicFollowApp, self).__init__(agent_config, moat_config)

    def dump(self, ev: str) -> None:
        if not self.agent_gvh.is_leader:
            return

        d = {'iter': self.locals['i'],
             'time': rospy.get_rostime().to_nsec()}
        if ev == "GUpdate":
            d['map'] = self.read_from_shared('global_map', None)
            d['map'].show()
        else:
            d['map'] = self.locals['map']

        with open(self.name, 'a') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=['iter', 'time', 'map'])
            writer.writerow(d)

    def initialize_vars(self):
        rospy.loginfo("Initializing variables")
        self.locals['i'] = 1
        self.locals['map'] = GridMap()
        self.update_local_map()
        self.locals['newpoint'] = True
        self.agent_gvh.create_aw_var('global_map', type(GridMap), GridMap())
        self.initialize_lock('GUpdate')
        self.locals['obstacle'] = []  # obstacle list for path planner

    def loop_body(self):
        if self.locals['i'] > 5000:
            self.trystop()
            return
        self.locals['i'] += 1

        # NewPoint event
        if self.locals['newpoint']:
            rospy.loginfo("NewPoint")
            # Local map is updated with global map only when we need to pick a new point
            self.locals['map'] = self.locals['map'] + self.read_from_shared('global_map', None)
            self.dump("NewPoint")
            self.locals['path'] = pick_path_to_frontier(self.locals['map'],
                                                        self.agent_gvh.moat.position,
                                                        self.agent_gvh.moat.planner,
                                                        self.locals['obstacle'])
            if len(self.locals['path']) > 0:
                rospy.loginfo("Target position: " + str(self.locals['path'][-1]))
                self.agent_gvh.moat.follow_path(self.locals['path'])
                self.locals['newpoint'] = False
            else:
                rospy.logwarn("Cannot find any path to frontier from " + str(self.agent_gvh.moat.position))
                self.trystop()
            return

        # LUpdate event
        if not self.locals['newpoint'] and not self.agent_gvh.moat.reached:
            rospy.loginfo("LUpdate")
            self.locals['newpoint'] = False
            self.update_local_map()
            self.dump("LUpdate")
            return

        # GUpdate event
        if self.agent_gvh.moat.reached:
            rospy.loginfo("GUpdate")
            self.update_local_map()
            if self.lock('GUpdate'):
                self.agent_gvh.put('global_map', self.read_from_shared('global_map', None)+self.locals['map'])
                self.dump("GUpdate")
                self.unlock('GUpdate')
                self.locals['newpoint'] = True
            else:
                return

    def update_local_map(self):
        for _ in range(self.agent_gvh.moat.tsync.qsize()):
            try:
                ipos, iscan = self.agent_gvh.moat.tsync.get_nowait()
                empty_map = get_empty_map(ipos, iscan)
                obs_map = get_obstacle_map(ipos, iscan, self.locals['obstacle'])
                self.locals['map'] = self.locals['map'] + empty_map + obs_map
            except queue.Empty:
                pass


def get_obstacles(pos: Pos, scan: list) -> list:
    x = pos.x
    y = pos.y
    yaw = pos.yaw
    obstacles = []
    for distance, cur_angle in scan:
        if distance == float('inf'):
            continue
        obs_x = x + np.cos(yaw + cur_angle)*distance
        obs_y = y + np.sin(yaw + cur_angle)*distance
        obstacles.append((obs_x, obs_y))

    return obstacles


def get_obstacle_map(pos, scan, cur_obstacles) -> GridMap:
    ret_map = GridMap()
    count = 0
    for pos_x, pos_y in get_obstacles(pos, scan):
        obs_x, obs_y = GridMap.quantize(pos_x, pos_y)
        if len(cur_obstacles) == 0:
            obs_pos = CylObs(Pos(np.array([obs_x, obs_y, 0.0])), np.array([0.6]))
            cur_obstacles.append(obs_pos)
        else:
            flag = 0
            for cur_obs in cur_obstacles:
                if cur_obs.position.x == obs_x and cur_obs.position.y == obs_y:
                    flag = 1 
            if not flag:
                obs_pos = CylObs(Pos(np.array([obs_x, obs_y, 0.0])), np.array([0.6]))
                cur_obstacles.append(obs_pos)
        ret_map[obs_x][obs_y] = GridMap.OCCUPIED
        count += 1
    return ret_map


def get_empty_map(pos, scan) -> GridMap:
    ret_map = GridMap()

    # Get the begin and end of x and y inside LIDAR_RANGE radius
    # [x_beg, x_end)*[y_beg, y_end) should cover all possibly empty grids
    # TODO get LIDAR_RANGE from LaserScan message
    # FIXME we can consider less grids if Lidar angle min/max are taken into account
    x_beg, y_beg, x_end, y_end = \
        np.ceil([pos.x - LIDAR_RANGE,
                 pos.y - LIDAR_RANGE,
                 pos.x + LIDAR_RANGE,
                 pos.y + LIDAR_RANGE]).astype(int).tolist()

    # NOTE x_end and y_end are excluded
    angle_inc = 0.01  # TODO get LIDAR_RANGE and angle increment from LaserScan message
    scan_angles = [ang for _, ang in scan]

    def is_covered_by_scan(polar: Tuple[float, float]) -> bool:
        nonlocal scan, scan_angles
        rad, ang = polar
        if rad > LIDAR_RANGE:
            return False
        #  Binary search the angles because it is in ascending order
        i = bisect(scan_angles, _normalize_radians(ang))
        if i:
            distance, angle = scan[i-1]
            return abs(_normalize_radians(angle - ang)) < angle_inc and rad < distance
        return False

    for x in range(x_beg, x_end):
        for y in range(y_beg, y_end):
            # Check if all four corners are covered by Lidar scan
            corners = [(x, y), (x+1, y), (x, y+1), (x+1, y+1)]
            polars = [(np.sqrt((_x-pos.x)**2 + (_y-pos.y)**2),  # radius
                       np.arctan2((_y-pos.y), (_x-pos.x)) - pos.yaw)  # angle
                      for _x, _y in corners]
            if any(not is_covered_by_scan(p) for p in polars):
                continue
            # else: # All four corners are covered
            ret_map[x][y] = GridMap.EMPTY

    return ret_map


def pick_path_to_frontier(m: GridMap, pos: pos3d, planner, obstacle_list) -> List[pos3d]:
    curr_grid = GridMap.quantize(pos.x, pos.y)
    next_grids = m.get_frontier_grids(curr_grid)
    next_pos_list = [pos3d(n[0] + .5, n[1] + .5, 0, pos.yaw) for n in next_grids]

    # XXX BFS returns from closest to furthest. We can add random permutation here
    def pos_key(new_pos: pos3d) -> float:
        new_yaw = np.arctan2((new_pos.y - pos.y), (new_pos.x - pos.x))
        forward_diff = _normalize_radians(new_yaw - pos.yaw)
        backward_diff = _normalize_radians(forward_diff + np.pi)
        # Choose the smaller to compute cost
        ang = min(abs(forward_diff), abs(backward_diff))
        """ x^2 + (y - 5.5 + 0.5z)^2 = (0.55(z-10))^2  # Cost function based on an oblique cone
        <=> (x^2 + y^2 -11y)/0.0525 + ((y + 0.55)/0.105)^2 = (z - (y + 0.55)/0.105)^2
        <=> (r^2 -11y)/0.0525 + ((y + 0.55)/0.105)^2 = (z - (y + 0.55)/0.105)^2
        <=> (r^2 -11y)/0.0525 + (z_0)^2 = (z - z_0)^2
        <=> sqrt((r^2 -11y)/0.0525 + (z_0)^2) = |z - z_0|
        max z=10 when (x, y) = (0, 0.5)
        """
        rad_sq = (new_pos.x - pos.x) ** 2 + (new_pos.y - pos.y) ** 2
        y = np.sqrt(rad_sq) * np.sin(ang)
        z_0 = (y + 0.55)/0.105
        lhs = np.sqrt((rad_sq - 11*y)/0.0525 + z_0**2)
        assert z_0 > 0

        return abs(z_0 - lhs)

    next_pos_list.sort(key=pos_key)
    for next_pos in next_pos_list:
        # TODO add obstacles from map
        path = planner.find_path(pos, next_pos, obstacle_list=obstacle_list)
        if path:
            return path

    return []
