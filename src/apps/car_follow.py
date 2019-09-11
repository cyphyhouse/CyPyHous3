import time

from src.config.configs import AgentConfig, MoatConfig, default_car_moat_config
from src.harness.agentThread import AgentThread
from src.motion.moat_test_car import MoatTestCar
from src.motion.pos_types import pos3d


class BasicFollowApp(AgentThread):

    def __init__(self, agent_config: AgentConfig, moat_config: MoatConfig):
        super(BasicFollowApp, self).__init__(agent_config, moat_config)
        self.start()

    def initialize_vars(self):
        self.locals['dest1'] = pos3d(1.9, 1.7, 0.)
        self.locals['dest2'] = pos3d(-2.1, 1.2, 0.)
        self.locals['tries'] = 1
        self.locals['init'] = True
        self.locals['back'] = True

    def loop_body(self):
        if self.locals['tries'] < 10 :
            if self.locals['init']:
                self.agent_gvh.moat.goTo(self.locals['dest1'])
                self.locals['init'] = False
                return
            if self.agent_gvh.moat.reached:
                if self.locals['back']:
                    self.agent_gvh.moat.goTo(self.locals['dest2'])
                    self.locals['back'] = False
                else:
                    self.agent_gvh.moat.goTo(self.locals['dest1'])
                    self.locals['back'] = True
                self.locals['tries'] = self.locals['tries'] + 1
                return
        else:
            self.stop()
