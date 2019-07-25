import time

import numpy as np

from src.config.configs import AgentConfig, MoatConfig, default_qc_moat_config
from src.harness.agentThread import AgentThread
from src.motion.moat_test_drone import MoatTestDrone
from src.motion.pos import Pos


class BasicFollowApp(AgentThread):

    def __init__(self, agent_config: AgentConfig, moat_config: MoatConfig):
        super(BasicFollowApp, self).__init__(agent_config, moat_config)
        self.agent_gvh.moat = MoatTestDrone(moat_config)
        self.start()

    def initialize_vars(self):
        self.locals['dest1'] = Pos(np.array([2., 1., 1.]))
        self.locals['dest2'] = Pos(np.array([-2., 1., 1.]))
        self.locals['dest3'] = Pos(np.array([2., -1., 1.]))
        self.locals['tries'] = 1

    def loop_body(self):
        if self.locals['tries'] == 1:
            self.agent_gvh.moat.goTo(self.locals['dest1'])
            time.sleep(5)
            self.locals['tries'] = 2
            return
        if self.locals['tries'] == 2:
            self.agent_gvh.moat.goTo(self.locals['dest2'])
            self.locals['tries'] = 3
            time.sleep(5)
            return
        if self.locals['tries'] == 3:
            self.agent_gvh.moat.goTo(self.locals['dest3'])
            self.locals['tries'] = 4
            time.sleep(5)
            self.stop()
            return


m = default_qc_moat_config('cyphyhousecopter')
a = AgentConfig(1, 1, "", 2000)
app = BasicFollowApp(a, m)
