import time

from src.config.configs import AgentConfig, MoatConfig, default_car_moat_config
from src.motion.rrt_star import RRT
from src.harness.agentThread import AgentThread
from src.motion.moat_test_car import MoatTestCar
from src.motion.pos import pos3d


class BasicFollowApp(AgentThread):

    def __init__(self, agent_config: AgentConfig, moat_config: MoatConfig):
        super(BasicFollowApp, self).__init__(agent_config, moat_config)
        self.agent_gvh.moat = MoatTestCar(moat_config)
        self.start()

    def initialize_vars(self):
        self.locals['dest1'] = pos3d(2., 1., 0.)  # Pos(np.array([2., 1., 0.]))
        self.locals['dest2'] = pos3d(-2., 1., 0.)  # Pos(np.array([-2., 1., 0.]))
        self.locals['dest3'] = pos3d(2., -1., 0.)  # Pos(np.array([2., -1., 0.]))
        self.locals['tries'] = 1

    def loop_body(self):
        if self.locals['tries'] == 1:
            path = self.agent_gvh.moat.planner.find_path(self.locals['dest1'],self.locals['dest2'],[])
            print("first path from 2,1,0 to -2, 1 , 0",path)
            self.agent_gvh.moat.follow_path(path)
            time.sleep(2)
            self.locals['tries'] = 2
            return
        if self.locals['tries'] == 2:
            path = self.agent_gvh.moat.planner.find_path(self.locals['dest2'], self.locals['dest3'],[])
            print("first path from -2,1,0 to 2, -1 , 0",path)
            self.agent_gvh.moat.follow_path(path)
            time.sleep(2)
            self.locals['tries'] = 3

            return




m = default_car_moat_config('hotdec_car')
obstacles =
m.planner = RRT()
a = AgentConfig(1, 1, "", 2000)
app = BasicFollowApp(a, m)
