import time

from src.config.configs import AgentConfig
from src.harness.agentThread import AgentThread
from src.objects.udt import get_tasks


class TaskApp(AgentThread):

    def __init__(self, agent_config: AgentConfig):
        super(TaskApp, self).__init__(agent_config, None)
        self.start()

    def initialize_vars(self):
        self.initialize_lock('pick_route')
        self.agent_gvh.create_aw_var('tasks', list, [0 for i in get_tasks(taskfile='src/apps/tasks.txt')])
        self.locals['my_task'] = None
        self.locals['doing'] = False
        self.locals['obstacles'] = []

    def loop_body(self):
        pass