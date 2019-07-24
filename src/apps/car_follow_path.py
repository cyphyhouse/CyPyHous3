import time

from src.config.configs import AgentConfig, MoatConfig, default_car_moat_config
from src.harness.agentThread import AgentThread
from src.motion.moat_test_car import MoatTestCar
from src.motion.pos import pos3d, Obs
from src.motion.rrt_star import RRT


class BasicFollowApp(AgentThread):

    def __init__(self, agent_config: AgentConfig, moat_config: MoatConfig):
        super(BasicFollowApp, self).__init__(agent_config, moat_config)
        self.agent_gvh.moat = MoatTestCar(moat_config)
        self.start()

    def initialize_vars(self):
        self.locals['dest1'] = pos3d(2., 2., 0.)  # Pos(np.array([2., 1., 0.]))
        self.locals['dest2'] = pos3d(2., -2., 0.)  # Pos(np.array([-2., 1., 0.]))
        self.locals['dest3'] = pos3d(-2., -2., 0.)  # Pos(np.array([2., -1., 0.]))
        self.locals['dest4'] = pos3d(-2., 2., 0.)  # Pos(np.array([2., -1., 0.]))

        self.locals['tries'] = 1

    def loop_body(self):
        if self.locals['tries'] == 1 :
            print("going to point 1")

            path = self.agent_gvh.moat.planner.find_path(self.agent_gvh.moat.position, self.locals['dest2'], obstacles)
            if path is None:
                print("no path for 1 to 2 ")
                self.locals['tries'] = 2
                return
            print("first path from 2,1,0 to -2, 1 , 0", path)
            self.agent_gvh.moat.follow_path(path)
            time.sleep(2)
            self.locals['tries'] = 2
            return
        if self.locals['tries'] == 2 and self.agent_gvh.moat.reached:
            #print("position is", self.agent_gvh.moat.position)
           # path = self.agent_gvh.moat.planner.find_path(self.agent_gvh.moat.position, self.locals['dest3'], obstacles)
            #if path is None:
                #print("no path for 2 to 3 ")
                #self.locals['tries'] = 3
                #return
            #print("first path from -2,1,0 to 2, -1 , 0", path)
           # self.agent_gvh.moat.follow_path(path)
           # time.sleep(3)
            self.locals['tries'] = 3

            return
        if self.locals['tries'] == 3 and self.agent_gvh.moat.reached:
            print("going to point 2")

            path = self.agent_gvh.moat.planner.find_path(self.agent_gvh.moat.position, self.locals['dest4'], obstacles)
            print("first path from 2,1,0 to -2, 1 , 0", path)
            if path is None:
                print("no path for 3 to 4 ")
                self.locals['tries'] = 4
                return
            self.agent_gvh.moat.follow_path(path)
            time.sleep(3)
            self.locals['tries'] = 4
            return
        if self.locals['tries'] == 4 and self.agent_gvh.moat.reached:
            print("going to point 3")
            path = self.agent_gvh.moat.planner.find_path(self.agent_gvh.moat.position, self.locals['dest1'], obstacles)
            if path is None:
                print("no path for 1 to 2 ")
                self.locals['tries'] = 5
                return
            print("first path from -2,1,0 to 2, -1 , 0", path)
            self.agent_gvh.moat.follow_path(path)
            time.sleep(3)
            self.locals['tries'] = 5
            return
        if self.locals['tries'] == 5 and self.agent_gvh.moat.reached:
            print("going to point 4")

            path = self.agent_gvh.moat.planner.find_path(self.agent_gvh.moat.position, self.locals['dest2'], obstacles)
            if path is None:
                print("no path for 1 to 2 ")
                self.locals['tries'] = 6
                return
            print("first path from 2,1,0 to -2, 1 , 0", path)
            self.agent_gvh.moat.follow_path(path)
            time.sleep(2)
            self.locals['tries'] = 6
            return
        if self.locals['tries'] == 6 and self.agent_gvh.moat.reached:
            print("going to point 5")

            path = self.agent_gvh.moat.planner.find_path(self.agent_gvh.moat.position, self.locals['dest4'], obstacles)
            print("first path from 2,1,0 to -2, 1 , 0", path)
            if path is None:
                print("no path for 3 to 4 ")
                self.locals['tries'] = 8
                return
            self.agent_gvh.moat.follow_path(path)
            time.sleep(3)
            self.locals['tries'] = 8
            return
        if self.locals['tries'] == 8 and self.agent_gvh.moat.reached:
            print("going to point 6")
            path = self.agent_gvh.moat.planner.find_path(self.agent_gvh.moat.position, self.locals['dest1'], obstacles)
            if path is None:
                print("no path for 1 to 2 ")
                self.locals['tries'] = 9
                return
            print("first path from -2,1,0 to 2, -1 , 0", path)
            self.agent_gvh.moat.follow_path(path)
            time.sleep(3)
            self.locals['tries'] = 9
            return



m = default_car_moat_config('hotdec_car')
obstacles = [Obs(0, 0, 0.75),Obs(0,-2,0.75)]
m.planner = RRT(goal_sample_rate=20)
a = AgentConfig(1, 1, "", 2000)
app = BasicFollowApp(a, m)
