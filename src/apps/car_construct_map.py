from src.config.configs import AgentConfig, MoatConfig
from src.harness.agentThread import AgentThread
from src.motion.pos_types import pos3d, Pos  # TODO Choose only one of them

from typing import Tuple, Optional
import rospy

import numpy as np
import matplotlib.pyplot as plt


def _normalize_radians(rad: float) -> float:
    return (rad + np.pi) % (2 * np.pi) - np.pi


class GridMap:
    UNKNOWN = -1
    EMPTY = 0
    OCCUPIED = 1

    HALF_GRID_WIDTH = 12
    GRID_WIDTH = 2 * HALF_GRID_WIDTH
    HALF_ARENA_WIDTH = 10

    def __init__(self, shape=(GRID_WIDTH, GRID_WIDTH)):
        """ Map should initialized with -1"""
        self.__grid_map = np.full(shape=shape, fill_value=GridMap.OCCUPIED, dtype=np.int8)
        for x in range(-GridMap.HALF_ARENA_WIDTH, GridMap.HALF_ARENA_WIDTH):
            for y in range(-GridMap.HALF_ARENA_WIDTH, GridMap.HALF_ARENA_WIDTH):
                self.__grid_map[x][y] = GridMap.UNKNOWN

    def __add__(self, other):
        """ Merge maps by choosing the larger value """
        assert self.__grid_map.shape == other.__grid_map.shape \
            and self.__grid_map.dtype == other.__grid_map.dtype
        ret = GridMap(self.__grid_map.shape)
        np.maximum(self.__grid_map, other.__grid_map, out=ret.__grid_map)
        return ret

    def __getitem__(self, key):
        return self.__grid_map.__getitem__(key)

    def __setitem__(self, key):
        return self.__grid_map.__setitem__(key)

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
    def to_2d_grid(x: float, y: float) -> Tuple[int, int]:
        return tuple(np.floor([x, y]).astype(int))

    def pick_next_grid(self, curr_grid: Tuple[int, int], curr_yaw: float,
                       yaw_diff_threshold: float = np.pi / 4) -> Optional[Tuple[int, int]]:
        """ Randomly pick an unoccupied adjacent grid within yaw threshold.
        :param curr_grid: Current grid of the device
        :param curr_yaw: Current orientation of the device
        :param yaw_diff_threshold: Threshold for orientation difference
        :return: An empty adjacent grid or None if cannot find one
        """
        assert self.__grid_map[curr_grid] != GridMap.OCCUPIED
        assert -np.pi < curr_yaw <= np.pi
        assert yaw_diff_threshold >= 0.0

        next_grid = None
        moves = [(_x, _y) for _x in [-1, 0, 1] for _y in [-1, 0, 1] if not (_x == 0 and _y == 0)]
        for x, y in np.random.permutation(moves):
            try_grid = (curr_grid[0] + x, curr_grid[1] + y)
            if self.__grid_map[try_grid] == GridMap.OCCUPIED:
                continue  # The grid is already occupied

            headings = [curr_yaw, curr_yaw + np.pi]  # Can go forward or backward
            yaw = np.arctan2(y, x)  # Target yaw
            norm_diffs = [_normalize_radians(h - yaw) for h in headings]
            yaw_diff = min(abs(d) for d in norm_diffs)
            if yaw_diff > yaw_diff_threshold:
                continue
            # else:
            next_grid = try_grid
            break

        assert next_grid != curr_grid, str(next_grid)
        return next_grid


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

    def loop_body(self):
        rospy.sleep(0.01)
        if self.locals['i'] > 40:
            self.trystop()
            return
        
        self.locals['i'] += 1
        # self.locals['map'].show()

        # NewPoint event
        if self.locals['newpoint']:
            print("Newpoint")
            # self.locals['map'] = self.locals['map'] # TODO merge global and local map
            self.locals['map'] = self.locals['map'] + self.read_from_shared('global_map', None)
            pos = self.agent_gvh.moat.position
            next_pos = pick_free_grid(self.locals['map'], pos)
            if not next_pos:
                return
            self.locals['path'] = self.agent_gvh.moat.planner.find_path(pos, next_pos)
            self.agent_gvh.moat.follow_path(self.locals['path'])
            self.locals['newpoint'] = False
            return

        # LUpdate event
        if not self.locals['newpoint'] and not self.agent_gvh.moat.reached:
            print("LUdate")
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
        pscan = tsync(self.agent_gvh.moat.tpos, self.agent_gvh.moat.tscan)
        self.agent_gvh.moat.tpos = {}
        for time in pscan:
            ipos, iscan = pscan[time]
            empty_map = get_empty_map(ipos, iscan)
            obs_map = get_obstacle_map(ipos, iscan)
            self.locals['map'] = self.locals['map'] + empty_map + obs_map


def global_grid(pos: pos3d, cur_angle: float, distance: float = 5.0) -> Tuple[int, int]:
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


def get_obstacle_map(pos, scan) -> GridMap:
    ret_map = GridMap()
    for pos_x, pos_y in get_obstacles(pos, scan):
        obs_x, obs_y = GridMap.to_2d_grid(pos_x, pos_y)
        ret_map[obs_x][obs_y] = GridMap.OCCUPIED
    return ret_map


def get_empty_map(pos, scan) -> GridMap:
    ret_map = GridMap()
    px, py = GridMap.to_2d_grid(pos.x, pos.y)
    yaw = pos.yaw

    left_point = scan[-1]
    right_point = scan[0]
    mid_point = scan[int(len(scan)/2)]

    lx, ly = global_grid(pos, left_point[1])
    rx, ry = global_grid(pos, right_point[1])
    mx, my = global_grid(pos, mid_point[1])

    # Left half
    if lx < mx:
        itrx = range(lx, mx+1)
        if ly > my:
            itry = range(ly+1, my-1, -1)
        else:
            itry = range(ly, my+1)
    else:
        itrx = range(mx, lx)
        if ly > my:
            itry = range(my, ly+1)
        else:
            itry = range(my+1, ly-1, -1)

    for x in itrx:
        for y in itry:
            if dist(x, y, px, py, yaw, scan):
                ret_map[x][y] = GridMap.EMPTY

    # Right half
    if rx < mx:
        itrx = range(rx, mx)
        if ry > my:
            itry = range(ry+1, my-1, -1)
        else:
            itry = range(ry, my+1)
    else:
        itrx = range(mx, rx)
        if ry > my:
            itry = range(my, ry+1)
        else:
            itry = range(my+1, ry-1, -1)

    for x in itrx:
        for y in itry:
            if dist(x, y, px, py, yaw, scan):
                ret_map[x][y] = GridMap.EMPTY

    return ret_map


def tsync(tpos: dict, tscan: dict) -> dict:
    pscan = {}
    for pt in tpos:
        for st in tscan:
            if abs(pt-st)<0.02:
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


def pick_free_grid(m: GridMap, pos: pos3d) -> Optional[pos3d]:
    curr_grid = GridMap.to_2d_grid(pos.x, pos.y)
    next_grid = m.pick_next_grid(curr_grid, pos.yaw)
    if next_grid:
        rospy.loginfo("Picked grid: " + str(next_grid))
        return pos3d(next_grid[0] + .5, next_grid[1] + .5, 0, pos.yaw)
    else:
        rospy.logwarn("Cannot find an unoccupied adjacent grid from " + str(pos))
        return None
