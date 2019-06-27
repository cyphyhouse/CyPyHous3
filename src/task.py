import time

import basic_synchronizer
from agentThread import AgentThread
from base_mutex import BaseMutex
from comm_handler import CommHandler, CommTimeoutError
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
        agent_gvh.port_list = [2000, 2001, 2002]
        if pid == 0:
            agent_gvh.is_leader = True
        mutex_handler = BaseMutexHandler(agent_gvh.is_leader, pid)
        agent_gvh.mutex_handler = mutex_handler
        agent_comm_handler = CommHandler(receiver_ip, r_port)
        super(AgentCreation, self).__init__(agent_gvh, agent_comm_handler, mutex_handler)

        self.agent_comm_handler.agent_gvh = self.agent_gvh
        self.agent_gvh.synchronizer = basic_synchronizer.BasicSynchronizer(pid, participants, [2000, 2001, 2002])

        self.rounds = 3
        self.start()

    def run(self):
        tasks = [Task((0, 1), 1, False, None), Task((0, 0), 2, False, None)]
        self.agent_gvh.create_aw_var('tasks', list, tasks)
        a = BaseMutex(1, [2000, 2001, 2002])
        self.agent_gvh.mutex_handler.add_mutex(a)
        requested = False
        a.agent_comm_handler = self.agent_comm_handler
        while not self.stopped():
            time.sleep(0.4)
            self.agent_gvh.flush_msgs()
            self.agent_comm_handler.handle_msgs()

            time.sleep(0.1)

            try:
                if not requested:
                    a.request_mutex()
                    requested = True
                else:

                    if self.agent_gvh.mutex_handler.has_mutex(a.mutex_id):
                        tasks = self.agent_gvh.get('tasks')
                        for task in tasks:
                            if not task.assigned:
                                task.assigned = True
                                task.assigned_to = self.pid()
                                print("assigned task", task.id, "to ", self.pid())
                                break
                        self.agent_gvh.put('tasks', tasks)
                        time.sleep(0.4)
                        self.rounds -= 1
                        requested = False
                        a.release_mutex()
                    else:
                        continue

                if self.rounds <= 0:
                    if not self.agent_gvh.is_leader:
                        self.stop()

                if not self.agent_comm_handler.is_alive():
                    self.stop()

            except CommTimeoutError:
                print("timed out on communication")
                self.stop()


b = AgentCreation(1, 3, "", 2001)
c = AgentCreation(2, 3, "", 2002)
a = AgentCreation(0, 3, "", 2000)
