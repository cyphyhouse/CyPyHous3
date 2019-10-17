from src.config.configs import AgentConfig, MoatConfig
from src.harness.agentThread import AgentThread
from src.motion.pos_types import pos3d, Pos  # TODO Choose only one of them

import numpy as np
import matplotlib.pyplot as plt
from math import floor


class GridMap:
    def __init__(self, shape=(200, 200)):
        """ Map should initialized with -1"""
        self.__grid_map = np.full(shape=shape, fill_value=-1, dtype=np.int8)

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
        plt.imshow(self.__grid_map)
        plt.pause(0.5)

class BasicFollowApp(AgentThread):

    def __init__(self, agent_config: AgentConfig, moat_config: MoatConfig):
        super(BasicFollowApp, self).__init__(agent_config, moat_config)

    def initialize_vars(self):
        self.locals['dest1'] = pos3d(2., 1., 0.)
        self.locals['dest2'] = pos3d(-2., 1., 0.)
        self.locals['dest3'] = pos3d(2., -1., 0.)
        self.locals['tries'] = 1
        self.locals['i'] = 1
        self.locals['map'] = GridMap(shape=(20,20))

    def loop_body(self):
        while (self.locals['i'] < 5):
            self.LUpdate()
            if self.locals['tries'] == 1:
                self.agent_gvh.moat.goTo(self.locals['dest1'])
                self.locals['tries'] = 2
                return
            if self.locals['tries'] == 2 and self.agent_gvh.moat.reached:
                self.agent_gvh.moat.goTo(self.locals['dest2'])
                self.locals['tries'] = 3
                return
            if self.locals['tries'] == 3 and self.agent_gvh.moat.reached:
                self.agent_gvh.moat.goTo(self.locals['dest3'])
                self.locals['tries'] = 2
                self.locals['i'] += 1
                return

    def LUpdate(self) -> None:
        pscan = self.tsync(self.agent_gvh.moat.tpos, self.agent_gvh.moat.tscan)
        self.agent_gvh.moat.tpos = {}
        for time in pscan:
            ipos, iscan = pscan[time]
            x = ipos.x
            y = ipos.y
            self.setAll(ipos, iscan)

            obstacles = self.getObs(ipos, iscan)

            for obs in obstacles:
                obsX = floor(obs[0]*1 + 9)
                obsY = floor(obs[1]*1 + 9)
                self.locals['map'][obsY][obsX] = 1

        # plt.imshow(self.locals['map'])
        # plt.pause(0.5)
        self.locals['map'].show()

    def getObs(self, ipos: Pos, iscan: list) -> list:
        x = ipos.x
        y = ipos.y
        yaw = ipos.yaw
        obstacles = []
        for distance, cur_angle in iscan:
            if distance == float('inf'):
                continue
            obsX = x + np.sin(np.pi/2 - yaw + cur_angle)*distance*np.cos(0.05)
            obsY = y + np.cos(np.pi/2 - yaw + cur_angle)*distance*np.cos(0.05)
            obstacles.append((obsX, obsY))

        return obstacles

    def tsync(self, tpos: dict, tscan: dict) -> dict:
        pscan = {}
        for pt in tpos:
            for st in tscan:
                if abs(pt-st)<0.01:
                    pscan[pt] = (tpos[pt], tscan[st])
                    break

        return pscan
        
    def setAll(self, pos, scan):
        px = floor(pos.x*1 + 9)
        py = floor(pos.y*1 + 9) 
        yaw = pos.yaw

        leftPoint = scan[-1]
        rightPoint = scan[0]
        midPoint = scan[int(len(scan)/2)]

        _, cur_angle = leftPoint
        distance = 5
        lx = floor((pos.x + np.sin(np.pi/2 - yaw + cur_angle)*distance*np.cos(0.05))*1 + 9)
        ly = floor((pos.y + np.cos(np.pi/2 - yaw + cur_angle)*distance*np.cos(0.05))*1 + 9)

        _, cur_angle = rightPoint
        distance = 5
        rx = floor((pos.x + np.sin(np.pi/2 - yaw + cur_angle)*distance*np.cos(0.05))*1 + 9)
        ry = floor((pos.y + np.cos(np.pi/2 - yaw + cur_angle)*distance*np.cos(0.05))*1 + 9)

        _, cur_angle = midPoint
        distance = 5
        mx = floor((pos.x + np.sin(np.pi/2 - yaw + cur_angle)*distance*np.cos(0.05))*1 + 9)
        my = floor((pos.y + np.cos(np.pi/2 - yaw + cur_angle)*distance*np.cos(0.05))*1 + 9)



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
                if self.dist(x,y,px,py,yaw,scan):
                    if self.locals['map'][y][x] == 1:
                        continue
                    self.locals['map'][y][x] = 0


        # Right half
        if rx < mx:
            itrx = range(rx, mx)
            if ry > my:
                itry = range(ry+1, my-1, -1)
            else:
                itry = range(ry, my+1)
        else:
            itr = range(mx, rx)
            if ry > my:
                itry = range(my, ry+1)
            else:
                itry = range(my+1, ry-1, -1)
        
        for x in itrx   :
            for y in itry:
                if self.dist(x,y,px,py, yaw, scan):
                    if self.locals['map'][y][x] == 1:
                        continue
                    self.locals['map'][y][x] = 0


    def dist(self, x1, y1, x2, y2, yaw, scan) -> bool:
        min_angle = -1.57079994678
        max_angle = 1.57079994678
        angle_inc = 0.00290888897143
        d = np.sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))
        try:
            delta = np.arctan((y1-y2)/(x1-x2))
        except ZeroDivisionError:
            if y1 > y2:
                delta = np.pi/2
            else:
                delta = -np.pi/2
        omega = yaw - delta

        for distance, angle in scan:
            if abs(angle - omega) < 0.01:
                if distance == float('inf'):
                    return d < 50
                else:
                    return d < distance #* 10
