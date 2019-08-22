import time

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
        self.locals['tasks'] = []
        self.locals['obstacles'] = []

    def loop_body(self):
        pass
