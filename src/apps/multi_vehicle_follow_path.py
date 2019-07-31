import time

from src.config.configs import AgentConfig, MoatConfig
from src.harness.agentThread import AgentThread
from src.motion.pos import pos3d, Pos


class BasicFollowApp(AgentThread):

    def __init__(self, agent_config: AgentConfig, moat_config: MoatConfig):
        super(BasicFollowApp, self).__init__(agent_config, moat_config)
        self.start()

    def initialize_vars(self):
        self.create_ar_var('pos', Pos, self.agent_gvh.moat.position)
        self.create_aw_var('pointnum', list, [0, 0, 0, 0, 0, 0, 0])
        self.initialize_lock('singlelock')
        self.locals['current_dest'] = -1
        self.locals['obstacles'] = []
        self.locals['dest'] = [pos3d(2., 2., 1.), pos3d(1., 1., 0), pos3d(2., 2., 0), pos3d(2., -2., 1.),
                               pos3d(-2., -2., 1.), pos3d(-2., 2., 1.), pos3d(-2, 1, 0)]
        self.locals['going'] = False
        self.locals['path'] = None

    def loop_body(self):
        time.sleep(1)
        other_vehicle = 0
        if self.pid() == 0:
            other_vehicle = 1

        if sum(self.read_from_shared('pointnum', None)) == len(self.locals['dest']):
            self.stop()
            return

        if not self.lock('singlelock'):
            return

        if not self.locals['going']:
            for i in range(len(self.read_from_shared('pointnum', None))):
                if self.read_from_shared('pointnum', None)[i] == 0:
                    self.locals['current_dest'] = i
                else:
                    continue

                self.locals['path'] = self.agent_gvh.moat.planner.find_path(self.agent_gvh.moat.position,
                                                                            self.locals['dest'][
                                                                                self.locals['current_dest']],
                                                                            self.locals['obstacles'])
                if self.locals['path'] is None:
                    print("no path for current point, trying next ")
                else:
                    break

            if self.locals['path'] is None:
                self.unlock('singlelock')
                print('no path found')
                return

            print("path is", self.locals['path'])
            self.locals['pointnum'] = self.read_from_shared('pointnum',None)
            self.write_to_shared('pointnum',None, self.locals['pointnum'])
            self.agent_gvh.moat.follow_path(self.locals['path'])

            # self.agent_gvh.moat.goTo(self.locals['dest'][self.read_from_shared('pointnum', None)])
            self.locals['going'] = True

        if self.agent_gvh.moat.reached:
            print("here, next point")
            self.write_to_shared('pos', self.pid(), self.agent_gvh.moat.position)

            time.sleep(0.1)
            self.locals['going'] = False
            self.unlock('singlelock')
