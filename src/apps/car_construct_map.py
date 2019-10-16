from src.config.configs import AgentConfig, MoatConfig
from src.harness.agentThread import AgentThread
from src.motion.pos_types import pos3d, Pos  # TODO Choose only one of them

import numpy as np
import matplotlib.pyplot as plt


class BasicFollowApp(AgentThread):

    def __init__(self, agent_config: AgentConfig, moat_config: MoatConfig):
        super(BasicFollowApp, self).__init__(agent_config, moat_config)

    def initialize_vars(self):
        self.locals['dest1'] = pos3d(2., 1., 0.)
        self.locals['dest2'] = pos3d(-2., 1., 0.)
        self.locals['dest3'] = pos3d(2., -1., 0.)
        self.locals['tries'] = 1
        self.locals['i'] = 1
        self.locals['map'] = np.zeros(shape=(2000,2000))

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
            # self.locals['map'][y][x] = 0
            #TODO setAll(pos, scan, dim)

            obstacles = self.getObs(ipos, iscan)

            for obs in obstacles:
                obsX = int(obs[0]*100 + 900)
                obsY = int(obs[1]*100 + 900)
                self.locals['map'][obsY][obsX] = 1

        plt.imshow(self.locals['map'])
        plt.pause(0.5)

    def getObs(self, ipos: Pos, iscan: list) -> list:
        x = ipos.x
        y = ipos.y
        yaw = ipos.yaw
        obstacles = []
        for distance, cur_angle in iscan:
            obsX = x + np.sin(np.pi/2 - yaw + cur_angle)*distance*np.cos(0.05)
            obsY = y + np.cos(np.pi/2 - yaw + cur_angle)*distance*np.cos(0.05)
            obstacles.append((obsX, obsY))

        return obstacles

    def tsync(self, tpos: dict, tscan: dict) -> dict:
        pscan = {}
        for pt in tpos:
            for st in tscan:
                if abs(pt-st)<0.01:
                    pscan[int(pt)] = (tpos[pt], tscan[st])
                    break

        return pscan
