import time

import src.CyPyHous3.src.motion.demo_planner as demo_planner
import src.CyPyHous3.src.motion.motionAutomaton_car as ma
from src.CyPyHous3.src.functionality.mutex_handler import BaseMutexHandler
from src.CyPyHous3.src.harness.agentThread import AgentThread
from src.CyPyHous3.src.harness.comm_handler import CommHandler, CommTimeoutError
from src.CyPyHous3.src.harness.gvh import Gvh
from src.CyPyHous3.src.motion.demo_planner import vec
from src.CyPyHous3.src.objects.base_mutex import BaseMutex


class Task(object):

    def __init__(self, location, id, assigned, assigned_to):
        self.location = location
        self.id = id
        self.assigned = assigned
        self.assigned_to = assigned_to

    def __repr__(self):
        return (str(self.id) + " assigned to " + str(self.assigned_to))


class AgentCreation(AgentThread):

    def __init__(self, pid, participants, receiver_ip, r_port, bot_name):

        agent_gvh = Gvh(pid, participants)
        moat = ma.MotionAutomaton(demo_planner.DemoPlan(), pid, bot_name, 10, 1)
        # moat = mb.MotionAutomaton(drone_planner.DPLAN(), pid, bot_name, 10, 0)
        agent_gvh.moat = moat
        agent_gvh.port_list = [2000]

        if pid == 0:
            agent_gvh.is_leader = True
        mutex_handler = BaseMutexHandler(agent_gvh.is_leader, pid)
        agent_gvh.mutex_handler = mutex_handler
        agent_comm_handler = CommHandler(receiver_ip, r_port, agent_gvh, 10)
        super(AgentCreation, self).__init__(agent_gvh, agent_comm_handler, mutex_handler)

        self.agent_comm_handler.agent_gvh = self.agent_gvh

        self.rounds = 10
        self.start()

    def run(self):

        if self.agent_gvh.moat.bot_type == 0:
            self.agent_gvh.moat.takeoff()
        tasks = get_tasks()
        route = []  # [vec(self.agent_gvh.moat.position.position.x, self.agent_gvh.moat.position.position.y, self.agent_gvh.moat.position.position.z)]

        self.agent_gvh.create_aw_var('tasks', list, tasks)
        self.agent_gvh.create_ar_var('route', list, route)
        self.agent_gvh.put('route', [[Pos(self.agent_gvh.moat.position.position.x,
                                          self.agent_gvh.moat.position.position.y,
                                          self.agent_gvh.moat.position.position.z)]], self.pid())
        # print(self.agent_gvh.get('route'))

        a = BaseMutex(1, [2000])

        self.agent_gvh.mutex_handler.add_mutex(a)
        a.agent_comm_handler = self.agent_comm_handler

        req_num = 0
        mytask = None

        while not self.stopped():
            print("current task status", [l.assigned for l in tasks])

            time.sleep(0.6)
            self.agent_gvh.flush_msgs()
            self.agent_comm_handler.handle_msgs()

            time.sleep(0.1)

            try:
                if mytask is not None and not self.agent_gvh.moat.reached:
                    print('pub a reached msg to prog')
                    continue
                elif mytask is not None and self.agent_gvh.moat.reached:
                    self.agent_gvh.put('route', [[vec(self.agent_gvh.moat.position.position.x,
                                                      self.agent_gvh.moat.position.position.y,
                                                      self.agent_gvh.moat.position.position.z)]], self.pid())

                    mytask = None

                test = self.agent_gvh.mutex_handler.has_mutex(a.mutex_id)

                # print("has mutex is", test)
                if not test:
                    # print(req_num)
                    a.request_mutex(req_num)
                    # print("requesting")


                else:
                    print("have mutex at", time.time())
                    tasks = self.agent_gvh.get('tasks')
                    route = self.agent_gvh.get('route')
                    for i in range(len(tasks)):
                        if not tasks[i].assigned:

                            # print("assigning task", tasks[i].id, "to ", self.pid())
                            mytask = tasks[i]
                            if mytask.location.position.z > 0 and self.agent_gvh.moat.bot_type == 1:
                                continue
                            if mytask.location.position.z <= 0 and self.agent_gvh.moat.bot_type == 0:
                                continue
                            # print("planner is", self.agent_gvh.moat.planner)
                            print(mytask.location)

                            self.agent_gvh.moat.planner.plan([self.agent_gvh.moat.position.position.x,
                                                              self.agent_gvh.moat.position.position.y,
                                                              self.agent_gvh.moat.position.position.z],
                                                             [mytask.location.position.x, mytask.location.position.y,
                                                              mytask.location.position.z])
                            testroute = self.agent_gvh.moat.planner.Planning()
                            # print('route is', testroute)
                            if demo_planner.clear_path(route, testroute, self.pid()):
                                print("cleared path")
                                tasks[i].assigned = True
                                tasks[i].assigned_to = self.pid()
                                route = testroute
                                self.agent_gvh.put('tasks', tasks)
                                self.agent_gvh.put('route', route, self.pid())
                                self.agent_gvh.moat.follow_path(testroute)
                            else:
                                print('route is not clear')
                                self.agent_gvh.put('route', [[vec(self.agent_gvh.moat.position.position.x,
                                                                  self.agent_gvh.moat.position.position.y,
                                                                  self.agent_gvh.moat.position.position.z)]],
                                                   self.pid())

                                mytask = None
                                continue

                            # print(testroute)
                            # print("just assigned mytask", mytask)

                            # self.agent_gvh.moat.goTo(tasks[i].location)

                            a.release_mutex()
                            break

                    time.sleep(0.4)
                    self.rounds -= 1
                    req_num = req_num + 1
                    if all(task.assigned for task in tasks):
                        if mytask is None:
                            self.stop()
                            continue

                        elif mytask is not None and self.agent_gvh.moat.reached:
                            self.stop()
                            continue
                        else:
                            continue

                if self.rounds <= 0:
                    if not self.agent_gvh.is_leader:
                        self.stop()

                if not self.agent_gvh.is_alive:
                    self.stop()

            except CommTimeoutError:
                print("timed out on communication")
                self.stop()


def get_tasks(taskfile='tasks.txt', repeat=1):
    from geometry_msgs.msg import Pose
    tasks = []
    tasklocs = open(taskfile, "r").readlines()
    for i in range(len(tasklocs)):
        locxyz = tasklocs[i].split(',')
        locnew = Pose()
        locnew.position.x, locnew.position.y, locnew.position.z = float(locxyz[0]), float(locxyz[1]), float(locxyz[2])
        tasks.append(Task(locnew, i, False, None))
    return tasks
