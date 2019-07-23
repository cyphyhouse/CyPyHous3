import time

from src.config.configs import AgentConfig, MoatConfig
from src.config.configs import BotType
from src.harness.agentThread import AgentThread
from src.motion.deconflict import clear_path
from src.objects.udt import get_tasks


class TaskApp(AgentThread):

    def __init__(self, agent_config: AgentConfig, moat_config: MoatConfig):
        super(TaskApp, self).__init__(agent_config, moat_config)
        self.rounds = 10
        self.start()

    def initialize_vars(self):
        self.initialize_lock('pick_route')
        self.agent_gvh.create_aw_var('tasks', list, get_tasks(taskfile='tasks.txt'))
        self.agent_gvh.create_ar_var('route', list, [])
        self.agent_gvh.put('route', [self.agent_gvh.moat.position], self.pid())
        self.locals['mytask'] = None

    def reset(self):
        if self.locals['mytask'] is not None and not self.agent_gvh.moat.reached:
            return False
        elif self.locals['mytask'] is not None and self.agent_gvh.moat.reached:
            self.agent_gvh.put('route', [self.agent_gvh.moat.position], self.pid())
            self.locals['mytask'] = None
            return True

    def loop_body(self):

        if self.reset():
            if self.lock('pick_route'):
                tasks = self.agent_gvh.get('tasks')
                route = self.agent_gvh.get('route')
                for i in range(len(tasks)):
                    if not tasks[i].assigned:
                        self.locals['mytask'] = tasks[i]

                        if self.locals['mytask'].location.z > 0 and self.agent_gvh.moat.bot_type == BotType.CAR:
                            continue
                        if self.locals['mytask'].location.z <= 0 and self.agent_gvh.moat.bot_type == BotType.QUAD:
                            continue

                        testroute = self.agent_gvh.moat.planner.find_path(self.agent_gvh.moat.position,
                                                                          self.locals['mytask'].location)
                        if clear_path(route, testroute, self.pid()):
                            tasks[i].assigned = True
                            tasks[i].assigned_to = self.pid()
                            route = testroute
                            self.agent_gvh.put('tasks', tasks)
                            self.agent_gvh.put('route', route, self.pid())
                            self.agent_gvh.moat.follow_path(testroute)
                        else:
                            self.agent_gvh.put('route', [self.agent_gvh.moat.position],
                                               self.pid())
                            self.locals['mytask'] = None
                            continue

                        self.unlock('pick_route')
                        break

                time.sleep(0.1)
                self.rounds -= 1

                if all(task.assigned for task in tasks):
                    if self.locals['mytask'] is None:
                        self.stop()
                        return

                    elif self.locals['mytask'] is not None and self.agent_gvh.moat.reached:
                        self.stop()
                        return
                    else:
                        return

            if self.rounds <= 0:
                if not self.agent_gvh.is_leader:
                    self.stop()

            if not self.agent_gvh.is_alive:
                self.stop()
