import time

from src.config.configs import AgentConfig
from src.harness.agentThread import AgentThread
from src.motion.pos_types import pos3d
from src.objects.udt import get_tasks


class TaskApp(AgentThread):

    def __init__(self, agent_config: AgentConfig):
        super(TaskApp, self).__init__(agent_config, None)
        self.start()

    def initialize_vars(self):
        self.initialize_lock('pick_route')
        self.agent_gvh.create_aw_var('tasks', list, [0 for i in get_tasks(taskfile='src/apps/tasks.txt')])
        self.agent_gvh.create_ar_var('route', list, [None])
        self.locals['my_task'] = None
        self.locals['doing'] = False
        self.locals['obstacles'] = []

    def loop_body(self):
        if not self.locals['doing']:
            if sum(self.read_from_shared('tasks', None)) == len(
                    self.read_from_shared('tasks', None)):
                self.stop()
                return

            if self.lock('pick_route'):
                time.sleep(1)
                self.locals['tasks'] = self.read_from_shared('tasks', None)
                # print("Tasks are", self.locals['tasks'])
                for i in range(len(self.locals['tasks'])):
                    if not self.locals['tasks'][i] == 1:
                        self.locals['my_task'] = self.locals['tasks'][i]
                        print("going to task", i)
                        self.locals['doing'] = True
                        self.locals['my_task'] = i
                        self.locals['tasks'][i] = 1
                        path = [pos3d(2., 1., 0.)]

                        self.agent_gvh.put('tasks', self.locals['tasks'])
                        self.agent_gvh.put('route', path, self.pid())
                        self.unlock('pick_route')
                        break
        else:
            if self.locals['my_task'] is not None:
                self.locals['my_task'] = None
            self.locals['doing'] = False
            return
