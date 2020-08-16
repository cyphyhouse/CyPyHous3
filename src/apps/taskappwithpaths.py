import time

from src.config.configs import AgentConfig, MoatConfig
from src.harness.agentThread import AgentThread
from src.objects.udt import get_tasks


class TaskApp(AgentThread):

    def __init__(self, agent_config: AgentConfig, moat_config: MoatConfig):
        super(TaskApp, self).__init__(agent_config, moat_config)
        self.start()

    def initialize_vars(self):
        self.initialize_lock('pick_route')
        self.locals['tasklist'] = get_tasks(taskfile='src/apps/tasks.txt')
        self.agent_gvh.create_aw_var('tasks', list, [0 for i in range(len(self.locals['tasklist']))])
        self.agent_gvh.create_ar_var('route', list, [self.moat.position])
        self.locals['my_task'] = None
        self.locals['test_route'] = None
        self.locals['doing'] = False
        self.locals['tasks'] = []
        self.locals['obstacles'] = []

    def loop_body(self):
        if not self.locals['doing']:
            if sum([int(a) for a in self.read_from_shared('tasks', None)]) == len(
                    self.read_from_shared('tasks', None)):
                self.stop()
                return

            if self.lock('pick_route'):
                time.sleep(0.2)
                self.locals['tasks'] = self.read_from_shared('tasks', None)
                print("assignment map is", self.locals['tasks'])
                for i in range(len(self.locals['tasks'])):
                    if not self.locals['tasks'][i] == 1:
                        self.locals['my_task'] = self.locals['tasklist'][i]

                        self.locals['test_route'] = self.moat.planner.find_path(self.moat.position,
                                                                                          self.locals[
                                                                                              'my_task'].location,
                                                                                          self.locals['obstacles'])

                        if self.locals['test_route'] is not None:
                            print("going to task", i)
                            self.locals['doing'] = True
                            self.locals['my_task'].assign(self.pid())
                            self.locals['tasklist'][i] = self.locals['my_task']
                            self.locals['tasks'][i] = 1
                            self.agent_gvh.put('tasks', self.locals['tasks'])
                            self.agent_gvh.put('route', self.locals['test_route'], self.pid())
                            # self.moat.follow_path(self.locals['test_route'])
                            break
                self.unlock('pick_route')

        else:
            if self.locals['my_task'] is not None:
                self.locals['my_task'] = None
            self.locals['doing'] = False
            return