from src.config.configs import AgentConfig, MoatConfig
from src.harness.agentThread import AgentThread
from src.motion.deconflict import clear_path
from src.objects.udt import get_tasks


class TaskApp(AgentThread):

    def __init__(self, agent_config: AgentConfig, moat_config: MoatConfig):
        super(TaskApp, self).__init__(agent_config, moat_config)
        self.start()

    def initialize_vars(self):
        self.initialize_lock('pick_route')
        self.agent_gvh.create_aw_var('tasks', list, get_tasks(taskfile='src/apps/tasks.txt'))
        self.agent_gvh.create_ar_var('route', list, [self.agent_gvh.moat.position])
        self.locals['my_task'] = None
        self.locals['test_route'] = None
        self.locals['doing'] = False

    def loop_body(self):

        if not self.locals['doing']:
            if sum([int(a.assigned) for a in self.read_from_shared('tasks', None)]) == len(
                    self.read_from_shared('tasks', None)):
                self.stop()
                return

            if self.lock('pick_route'):
                for i in range(len(self.read_from_shared('tasks', None))):
                    if not self.read_from_shared('tasks', None)[i].assigned:
                        self.locals['my_task'] = self.read_from_shared('tasks', None)[i]
                        self.locals['test_route'] = self.agent_gvh.moat.planner.find_path(self.agent_gvh.moat.position,
                                                                                          self.locals[
                                                                                              'my_task'].location, [])
                        if clear_path(self.read_from_shared('route', None), self.locals['test_route'], self.pid()):
                            self.locals['doing'] = True
                            self.read_from_shared('tasks', None)[i].assign(self.pid())
                            self.agent_gvh.put('tasks', self.read_from_shared('tasks', None))
                            self.agent_gvh.put('route', self.locals['test_route'], self.pid())
                            self.agent_gvh.moat.follow_path(self.locals['test_route'])
                        else:
                            self.agent_gvh.put('route', [self.agent_gvh.moat.position],
                                               self.pid())
                            self.locals['my_task'] = None
                            self.locals['doing'] = False
                            continue
                        self.unlock('pick_route')
                        break
        else:
            if self.agent_gvh.moat.reached:
                if self.locals['my_task'] is not None:
                    self.locals['my_task'] = None
                self.locals['doing'] = False
                return
