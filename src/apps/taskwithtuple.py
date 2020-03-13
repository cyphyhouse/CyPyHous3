import time

from src.config.configs import AgentConfig, MoatConfig
from src.harness.agentThread import AgentThread
from src.motion.deconflict import clear_path
from src.objects.udt import get_tasks
from src.motion.pos_types import pos3d

import numpy as np
from src.motion.pos_types import Pos
from src.motion.rectobs import RectObs
import rospy
from gazebo_msgs.msg import ModelStates

class TaskApp(AgentThread):

    def __init__(self, agent_config: AgentConfig, m):
        super(TaskApp, self).__init__(agent_config, m)
        self.start()

    def initialize_vars(self):
        self.initialize_lock('pick_route')
        self.agent_gvh.create_aw_var('tasks', list, [0 for i in get_tasks(taskfile='/home/mjiang24/Documents/CyPyHous3/src/apps/tasks.txt')])
        self.agent_gvh.create_ar_var('route', list, [self.agent_gvh.moat.position.to_list()])

        self.locals['my_task'] = None
        self.locals['doing'] = False
        self.locals['tasklist'] = get_tasks(taskfile='/home/mjiang24/Documents/CyPyHous3/src/apps/tasks.txt')
        self.locals['obstacles'] = {}
        self.locals['obstaclesUpdated'] = False

        self._obstacle_listener = rospy.Subscriber("/gazebo/model_states", ModelStates, self.updateObstacles)

    def loop_body(self):
        time.sleep(0.1)
        print("executing round", self.agent_gvh.round_num)
        if not self.locals['doing']:
            if sum(self.read_from_shared('tasks', None)) == len(
                    self.read_from_shared('tasks', None)):
                print("done all tasks")
                self.trystop()
                return

            if self.lock('pick_route'):
                print("i have mutex")
                self.locals['tasks'] = self.read_from_shared('tasks', None)
                #for i in range(self.num_agents()):
                #    print("route for agent",i," is :",self.read_from_shared('route',i))
                # print("Tasks are", self.locals['tasks'])
                print("obstacles are: ", self.locals['obstacles'])
                for i in range(len(self.locals['tasks'])):
                    if not self.locals['tasks'][i] == 1:
                        self.locals['doing'] = True
                        self.locals['my_task'] = self.locals['tasklist'][i]
                        self.locals['test_route'] = self.agent_gvh.moat.planner.find_path(self.agent_gvh.moat.position,
                                                                                          self.locals[
                                                                                              'my_task'].location,
                                                                                          self.locals['obstacles'].values())
                        if clear_path([[pos3d(*i) for i in path] for path in
                                       [self.read_from_shared('route', pid) for pid in range(self.num_agents())]],
                                      self.locals['test_route'], self.pid(), tolerance=1.00):
                            print("going to task", i)
                            self.locals['doing'] = True
                            self.locals['tasks'][i] = 1
                            self.sharePath()
                            break
                        else:
                            self.agent_gvh.put('route', [self.agent_gvh.moat.position.to_list()],
                                               self.pid())
                            self.locals['my_task'] = None
                            self.locals['doing'] = False
                            continue
                if not self.locals['doing']:
                    print("didnt find a clear path")

                self.unlock('pick_route')
                print("released mutex")
                time.sleep(0.05)
        else:
            print("doing old task")
            print(self.locals['obstaclesUpdated'])
            if self.locals['obstaclesUpdated']:
                self.agent_gvh.moat.stop()
                self.locals['obstaclesUpdated'] = False
                self.locals['test_route'] = self.agent_gvh.moat.planner.update_obstacles(self.locals['obstacles'].values(), self.agent_gvh.moat.position)
                if clear_path([[pos3d(*i) for i in path] for path in
                               [self.read_from_shared('route', pid) for pid in range(self.num_agents())]],
                              self.locals['test_route'], self.pid(), tolerance=1.00):
                    print("continue going to task")
                    self.locals['doing'] = True
                    self.sharePath()
                else:
                    self.agent_gvh.put('route', [self.agent_gvh.moat.position.to_list()],
                                       self.pid())
                    self.locals['my_task'] = None
                    self.locals['doing'] = False
            if self.agent_gvh.moat.reached:
                if self.locals['my_task'] is not None:
                    self.locals['my_task'] = None
                self.locals['doing'] = False
                return

    def sharePath(self):
        self.agent_gvh.put('tasks', self.locals['tasks'])
        self.agent_gvh.put('route', [i.to_list() for i in self.locals['test_route']], self.pid())
        self.agent_gvh.moat.follow_path(self.locals['test_route'])

    def updateObstacles(self, data):
        for name in data.name:
            if "cube" in name or "box" in name:
                index = data.name.index(name)

                obstacle = RectObs(Pos(np.array([data.pose[index].position.x, data.pose[index].position.y, 0])), np.array([1,1,1]))
                try:
                    if name not in self.locals['obstacles']:
                        self.locals['obstacles'][name] = obstacle
                    if obstacle not in self.locals['obstacles'].values():
                        # print(name, "obstacle has changed to ", obstacle.position)
                        # moved_dist = (obstacle.position-self.locals['obstacles'][name].position).magnitude()
                        # print(moved_dist)
                        self.locals['obstaclesUpdated'] = True
                        self.locals['obstacles'][name] = obstacle
                except KeyError:
                    pass

