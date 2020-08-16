from src.config.configs import AgentConfig, MoatConfig
from src.harness.agentThread import AgentThread
from src.motion.pos_types import Pos
import rospy

import numpy as np

class MapApp(AgentThread):

    def __init__(self, agent_config: AgentConfig, moat_config: MoatConfig):
        super(MapApp, self).__init__(agent_config, moat_config)

    def initialize_vars(self):
        self.locals['new_point'] = True
        pass

    def loop_body(self):
        rospy.sleep(0.1)
        pscan = tsync(self.moat.tpos, self.moat.tscan)
        print("|pscan|", len(pscan))        
        for pt in pscan:
            ipos, iscan = pscan[pt]
            obstacles = getObs(ipos, iscan)
            print("obstacles: ", obstacles)


        if self.locals['new_point']:
            self.locals['new_point'] = False
            return

        if not self.locals['new_point']:
            self.stop()
            return


def getObs(ipos: Pos, iscan: list) -> list:
    x = ipos.x
    y = ipos.y
    yaw = ipos.yaw
    obstacles = []
    for distance, cur_angle in iscan:
        obsX = x + np.sin(np.pi/2 - yaw + cur_angle)*distance*np.cos(0.05)
        obsY = y + np.cos(np.pi/2 - yaw + cur_angle)*distance*np.cos(0.05)
        obstacles.append((obsX, obsY))

    return obstacles


def tsync(tpos: dict, tscan: dict) -> dict:
    pscan = {}
    for pt in tpos:
        for st in tscan:
            if abs(pt-st) < 0.02:
                pscan[pt] = (tpos[pt], tscan[st])
                break

    return pscan

