from src.config.configs import AgentConfig, MoatConfig
from src.harness.agentThread import AgentThread
from src.motion.pos_types import pos3d, Pos  # TODO Choose only one of them
from src.motion.rectobs import RectObs
from src.motion.cylobs import CylObs

from collections import deque
from typing import List, Tuple, Optional
import rospy

import numpy as np
import matplotlib.pyplot as plt
import copy


def _normalize_radians(rad: float) -> float:
    return (rad + np.pi) % (2 * np.pi) - np.pi


Grid = Tuple[int, int]


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

    def initialize_vars(self):
        self.locals['i'] = 1
        self.locals['map'] = GridMap()
        self.update_local_map()
        self.locals['newpoint'] = True
        self.agent_gvh.create_aw_var('global_map', type(GridMap), GridMap())
        self.initialize_lock('GUpdate')
        self.locals['obstacle'] = [] # obstacle list for path planner

    def loop_body(self):
        rospy.sleep(0.01)
        if self.locals['i'] > 80:
            self.trystop()
            return
        
        self.locals['i'] += 1

        # NewPoint event
        if self.locals['newpoint']:
            print("Newpoint")
            # Local map is updated with global map only when we need to pick a new point
            self.locals['map'] = self.locals['map'] + self.read_from_shared('global_map', None)
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
            print("LUpdate")
            self.locals['newpoint'] = False
            self.update_local_map()
            return

        # GUpdate event
        if self.agent_gvh.moat.reached:
            print("GUpdate")
            self.update_local_map()
            if self.lock('GUpdate'):
                self.agent_gvh.put('global_map', self.read_from_shared('global_map', None)+self.locals['map'])
                self.read_from_shared('global_map', None).show()
                self.unlock('GUpdate')
                self.locals['newpoint'] = True
            else:
                return

    def update_local_map(self):
        pscan = self.agent_gvh.moat.tsync # tsync(self.agent_gvh.moat.tpos, self.agent_gvh.moat.tscan)
        self.agent_gvh.moat.tpos = {}
        for time in list(pscan):
            ipos, iscan = pscan[time]
            empty_map = get_empty_map(ipos, iscan)
            obs_map = get_obstacle_map(ipos, iscan, self.locals['obstacle'])
            self.locals['map'] = self.locals['map'] + empty_map + obs_map


def global_grid(pos: pos3d, cur_angle: float, distance: float = 5.0) -> Grid:
    x = np.floor((pos.x + np.sin(np.pi / 2 - pos.yaw + cur_angle) * distance * np.cos(0.05))).astype(int)
    y = np.floor((pos.y + np.cos(np.pi / 2 - pos.yaw + cur_angle) * distance * np.cos(0.05))).astype(int)
    return x, y


def get_obstacles(ipos: Pos, iscan: list) -> list:
    x = ipos.x
    y = ipos.y
    yaw = ipos.yaw
    obstacles = []
    for distance, cur_angle in iscan:
        if distance == float('inf'):
            continue
        obs_x = x + np.sin(np.pi/2 - yaw + cur_angle)*distance*np.cos(0.05)
        obs_y = y + np.cos(np.pi/2 - yaw + cur_angle)*distance*np.cos(0.05)
        obstacles.append((obs_x, obs_y))

    return obstacles


def get_obstacle_map(pos, scan, cur_obstalces) -> GridMap:
    ret_map = GridMap()
    obstacle_list = []
    count = 0
    for pos_x, pos_y in get_obstacles(pos, scan):
        obs_x, obs_y = GridMap.quantize(pos_x, pos_y)
        if len(cur_obstalces) == 0:
            obs_pos = CylObs(Pos(np.array([obs_x, obs_y, 0.0])), np.array([0.6]))
            cur_obstalces.append(obs_pos)
        else:
            flag = 0
            for cur_obs in cur_obstalces:
                if cur_obs.position.x == obs_x and cur_obs.position.y == obs_y:
                    flag = 1 
            if not flag:
                obs_pos = CylObs(Pos(np.array([obs_x, obs_y, 0.0])), np.array([0.6]))
                cur_obstalces.append(obs_pos)
        ret_map[obs_x][obs_y] = GridMap.OCCUPIED
        count += 1
    return ret_map


def get_empty_map(pos, scan) -> GridMap:
    ret_map = GridMap()
    px, py = GridMap.quantize(pos.x, pos.y)
    yaw = pos.yaw

    left_point = scan[-1]
    right_point = scan[0]
    mid_point = scan[int(len(scan)/2)]

    lx, ly = global_grid(pos, left_point[1])
    rx, ry = global_grid(pos, right_point[1])
    mx, my = global_grid(pos, mid_point[1])

    # Left half
    if lx < mx:
        itrx = range(lx-2, mx+2)
        if ly > my:
            itry = range(ly+2, my-2, -1)
        else:
            itry = range(ly-2, my+2)
    else:
        itrx = range(mx-2, lx+2)
        if ly > my:
            itry = range(my-2, ly+2)
        else:
            itry = range(my+2, ly-2, -1)
    for x in itrx:
        for y in itry:
            if dist(x, y, px, py, yaw, scan):
                ret_map[x][y] = GridMap.EMPTY

    # Right half
    if rx < mx:
        itrx = range(rx-2, mx+2)
        if ry > my:
            itry = range(ry+2, my-2, -1)
        else:
            itry = range(ry-2, my+2)
    else:
        itrx = range(mx-2, rx+2)
        if ry > my:
            itry = range(my-2, ry+2)
        else:
            itry = range(my+2, ry-2, -1)

    for x in itrx:
        for y in itry:
            if dist(x, y, px, py, yaw, scan):
                ret_map[x][y] = GridMap.EMPTY

    return ret_map


def tsync(tpos: dict, tscan: dict) -> dict:
    pscan = {}
    for pt in tpos:
        for st in tscan:
            print(pt, st)
            if abs(pt-st)<0.1:
                pscan[pt] = (tpos[pt], tscan[st])
                break

    return pscan


def dist(x1, y1, x2, y2, yaw, scan) -> bool:
    d = np.sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))

    delta = np.arctan2((y1-y2), (x1-x2))
    omega = yaw - delta

    for distance, angle in scan:
        if abs(angle - omega) < 0.01:
            if distance == float('inf'):
                return d < 5
            else:
                return d < distance


def pick_path_to_frontier(m: GridMap, pos: pos3d, planner, obstacle_list) -> List[pos3d]:
    curr_grid = GridMap.quantize(pos.x, pos.y)
    next_grids = m.get_frontier_grids(curr_grid)
    next_pos_list = [pos3d(n[0] + .5, n[1] + .5, 0, pos.yaw) for n in next_grids]
    # XXX reverse because BFS returns from closest to furthest. We can add random permutation here
    next_pos_list.reverse()
    for next_pos in next_pos_list:
        # TODO add obstacles from map
        path = planner.find_path(pos, next_pos, obstacle_list=obstacle_list)
        if path:
            return path

    return []
