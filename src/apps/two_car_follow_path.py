import time

from src.config.configs import AgentConfig, MoatConfig
from src.harness.agentThread import AgentThread
from src.motion.moat_test_car import MoatTestCar
from src.motion.pos import pos3d, Pos, Obs


class BasicFollowApp(AgentThread):

    def __init__(self, agent_config: AgentConfig, moat_config: MoatConfig):
        super(BasicFollowApp, self).__init__(agent_config, moat_config)
        self.agent_gvh.moat = MoatTestCar(moat_config)
        self.start()

    def initialize_vars(self):
        self.create_ar_var('carpos', Pos, self.agent_gvh.moat.position)
        self.create_aw_var('pointnum', int, 0)
        self.initialize_lock('singlelock')
        self.locals['dest'] = [pos3d(2., 2., 0.), pos3d(2., -2., 0.), pos3d(-2., -2., 0.), pos3d(-2., 2., 0.)]
        self.locals['obstacles'] = [Obs(0, 0, 0.75), None]

    def loop_body(self):
        other_car = 0
        if self.pid() == 0:
            other_car = 1
        self.locals['obstacles'][1] = self.read_from_shared('carpos', other_car)
        if self.lock('singlelock') and self.read_from_shared('pointnum') < 4:
            print("going to point ", self.read_from_shared('pointnum'))
            path = self.agent_gvh.moat.planner.find_path(self.agent_gvh.moat.position,
                                                         self.locals['dest'][self.read_from_shared('pointnum')],
                                                         self.locals['obstacles'])
            if path is None:
                print("no path for current point, sending to other car ")
                self.locals['tries'] = 2
                return
            print("path is", path)
            self.agent_gvh.moat.follow_path(path)
            time.sleep(2)
            if self.agent_gvh.moat.reached:
                self.write_to_shared('carpos', self.pid(), self.agent_gvh.moat.position)
                self.write_to_shared('pointnum', None, self.read_from_shared('pointnum') + 1)
                time.sleep(0.1)
                self.unlock('singlelock')
