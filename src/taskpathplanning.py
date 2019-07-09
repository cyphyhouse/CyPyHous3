import time

import basic_synchronizer
import motionAutomaton
import rrt_star
from agentThread import AgentThread
from base_mutex import BaseMutex
from comm_handler import CommHandler, CommTimeoutError
from geometry_msgs.msg import Pose
from gvh import Gvh
from mutex_handler import BaseMutexHandler


class Task(object):

    def __init__(self, location, id, assigned, assigned_to):
        self.location = location
        self.id = id
        self.assigned = assigned
        self.assigned_to = assigned_to

    def __repr__(self):
        return (str(self.id) + " assigned to " + str(self.assigned_to))


class AgentCreation(AgentThread):
    """
    test class to test that agent thread objects are created
    and fields are accessed safely and correctly .
    """

    def __init__(self, pid, participants, receiver_ip, r_port):
        """
        parameters to instantiate the gvh and communication handler.
        :param pid:
        :param participants:
        :param r_port:
        """
        agent_gvh = Gvh(pid, participants)
        moat = motionAutomaton.MotionAutomaton(rrt_star.RRT, pid, 'hotdec_car', 10)
        agent_gvh.moat = moat
        agent_gvh.port_list = [2000]
        if pid == 1:
            agent_gvh.is_leader = True
        mutex_handler = BaseMutexHandler(agent_gvh.is_leader, pid)
        agent_gvh.mutex_handler = mutex_handler
        agent_comm_handler = CommHandler(receiver_ip, r_port, agent_gvh, 10)
        super(AgentCreation, self).__init__(agent_gvh, agent_comm_handler, mutex_handler)

        self.agent_comm_handler.agent_gvh = self.agent_gvh
        self.agent_gvh.synchronizer = basic_synchronizer.BasicSynchronizer(pid, participants, [2000])
        self.agent_gvh.moat = motionAutomaton.MotionAutomaton(self.agent_gvh.pid, self.agent_gvh.bot_name)

        self.rounds = 10
        self.start()

    def run(self):
        b = Pose()
        b.position.x, b.position.y, b.position.z = 1.0, 1.0, 0.0
        c = Pose()
        c.position.x, c.position.y, c.position.z = 1.0, 2.0, 0.0
        d = Pose()
        d.position.x, d.position.y, d.position.z = 2.0, 1.0, 0.0

        tasks = [Task(b, 1, False, None), Task(c, 2, False, None), Task(d, 3, False, None)]
        route = []
        self.agent_gvh.create_aw_var('tasks', list, tasks)
        self.agent_gvh.create_ar_var('route', list, route)

        a = BaseMutex(1, [2000])
        self.agent_gvh.mutex_handler.add_mutex(a)
        a.agent_comm_handler = self.agent_comm_handler
        req_num = 0
        mytask = None

        while not self.stopped():
            print("working at this point")
            time.sleep(0.6)
            self.agent_gvh.flush_msgs()
            self.agent_comm_handler.handle_msgs()

            time.sleep(0.1)

            try:
                if mytask is not None and not self.agent_gvh.moat.reached:
                    continue
                elif mytask is not None and self.agent_gvh.moat.reached:
                    mytask = None

                test = self.agent_gvh.mutex_handler.has_mutex(a.mutex_id)
                # print("has mutex is", test)
                if not test:
                    # print(req_num)
                    a.request_mutex(req_num)
                    # print("requesting")
                else:
                    # print("have mutex at", time.time())
                    tasks = self.agent_gvh.get('tasks')
                    for i in range(len(tasks)):
                        if not tasks[i].assigned:
                            tasks[i].assigned = True
                            tasks[i].assigned_to = self.pid()
                            # print("assigned task", tasks[i].id, "to ", self.pid())
                            mytask = tasks[i]
                            print(self.agent_gvh.moat.planner)
                            testroute = self.agent_gvh.moat.find_path(mytask.location)

                            print(testroute)
                            # print("just assigned mytask", mytask)
                            self.agent_gvh.put('tasks', tasks)
                            self.agent_gvh.moat.follow_path(testroute)
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


a = AgentCreation(0, 1, "", 2000)
