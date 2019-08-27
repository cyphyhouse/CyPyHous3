import time

from src.config.configs import AgentConfig, MoatConfig
from src.harness.agentThread import AgentThread
from src.motion.deconflict import clear_path
from src.objects.udt import get_tasks
from src.motion.pos_types import pos3d


class TaskApp(AgentThread):

    def __init__(self, agent_config: AgentConfig, m):
        super(TaskApp, self).__init__(agent_config, m)
        self.start()

    def initialize_vars(self):
        self.initialize_lock('pick_route')
        self.agent_gvh.create_aw_var('tasks', list, [0 for i in get_tasks(taskfile='src/apps/tasks.txt')])
        self.agent_gvh.create_ar_var('route', list, [self.agent_gvh.moat.position.to_list()])

        self.locals['my_task'] = None
        self.locals['doing'] = False
        self.locals['tasklist'] = get_tasks(taskfile='src/apps/tasks.txt')
        self.locals['obstacles'] = []

    def loop_body(self):
        time.sleep(0.01)

        if not self.locals['doing']:
            if sum(self.read_from_shared('tasks', None)) == len(
                    self.read_from_shared('tasks', None)):
                print("done all tasks")
                self.trystop()
                return

            if self.lock('pick_route'):
                self.locals['tasks'] = self.read_from_shared('tasks', None)
                #print("the computed paths on round",self.agent_gvh.round_num,"are:\n")
                #for i in range(self.num_agents()):
                #    print("route for agent",i," is :",self.read_from_shared('route',i))
                # print("Tasks are", self.locals['tasks'])
                for i in range(len(self.locals['tasks'])):
                    if not self.locals['tasks'][i] == 1:
                        self.locals['doing'] = True
                        self.locals['my_task'] = self.locals['tasklist'][i]
                        self.locals['test_route'] = self.agent_gvh.moat.planner.find_path(self.agent_gvh.moat.position,
                                                                                          self.locals[
                                                                                              'my_task'].location,
                                                                                          [])
                        if clear_path([[pos3d(*i) for i in path] for path in
                                       [self.read_from_shared('route', pid) for pid in range(self.num_agents())]],
                                      self.locals['test_route'], self.pid(), tolerance=0.75):
                            print("going to task", i)
                            self.locals['doing'] = True
                            self.locals['tasks'][i] = 1
                            self.agent_gvh.put('tasks', self.locals['tasks'])
                            self.agent_gvh.put('route', [i.to_list() for i in self.locals['test_route']], self.pid())
                            self.agent_gvh.moat.follow_path(self.locals['test_route'])
                            break
                        else:
                            self.agent_gvh.put('route', [self.agent_gvh.moat.position.to_list()],
                                               self.pid())
                            self.locals['my_task'] = None
                            self.locals['doing'] = False
                            continue
                self.unlock('pick_route')
                time.sleep(0.01)
        else:
            if self.agent_gvh.moat.reached:
                if self.locals['my_task'] is not None:
                    self.locals['my_task'] = None
                self.locals['doing'] = False
                return