import time

from src.config.configs import AgentConfig, MoatConfig
from src.harness.agentThread import AgentThread
from src.motion.pos import pos3d, Pos, RoundObs


class BasicFollowApp(AgentThread):

    def __init__(self, agent_config: AgentConfig, moat_config: MoatConfig):
        super(BasicFollowApp, self).__init__(agent_config, moat_config)
        self.start()

    def initialize_vars(self):
        self.create_ar_var('pos', Pos, self.agent_gvh.moat.position)
        self.create_aw_var('pointnum', int, 0)
        self.initialize_lock('singlelock')
        self.locals['obstacles'] = [RoundObs(0., 2., 1., 1.)]
        self.locals['dest'] = [pos3d(2., 2., 1.), pos3d(1., 1., 0), pos3d(2., 2., 0), pos3d(2., -2., 1.),
                               pos3d(-2., -2., 1.), pos3d(-2., 2., 1.), pos3d(-2, 1, 0)]
        self.locals['going'] = False

    def loop_body(self):
        other_vehicle = 0
        if self.pid() == 0:
            other_vehicle = 1

        if self.read_from_shared('pointnum', None) > 6:
            self.stop()

        if not self.lock('singlelock'):
            return

        print("available point is", self.read_from_shared('pointnum', None))

        if not self.locals['going']:
            print("trying to go to point", self.read_from_shared('pointnum', None))

            path = self.agent_gvh.moat.planner.find_path(self.agent_gvh.moat.position,
                                                         self.locals['dest'][self.read_from_shared('pointnum', None)],
                                                         self.locals['obstacles'])
            if path is None:
                print("no path for current point, sending to other vehicle ")
                self.unlock('singlelock')
                return

            print("path is", path)
            self.agent_gvh.moat.follow_path(path)

            # self.agent_gvh.moat.goTo(self.locals['dest'][self.read_from_shared('pointnum', None)])
            self.locals['going'] = True

        if self.agent_gvh.moat.reached:
            self.write_to_shared('pos', self.pid(), self.agent_gvh.moat.position)
            self.write_to_shared('pointnum', None, self.read_from_shared('pointnum', None) + 1)
            time.sleep(0.1)
            self.locals['going'] = False
            self.unlock('singlelock')
