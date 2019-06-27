import time

from agentThread import AgentThread,send
from base_mutex import BaseMutex
from comm_handler import CommHandler, CommTimeoutError
from gvh import Gvh
from mutex_handler import BaseMutexHandler
import basic_synchronizer


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
        agent_gvh.port_list = [2000]
        if pid == 0:
            agent_gvh.is_leader = True
        mutex_handler = BaseMutexHandler(agent_gvh.is_leader, pid)
        agent_gvh.mutex_handler = mutex_handler
        agent_comm_handler = CommHandler(receiver_ip, r_port)
        super(AgentCreation, self).__init__(agent_gvh, agent_comm_handler, mutex_handler)

        self.agent_comm_handler.agent_gvh = self.agent_gvh
        self.agent_gvh.synchronizer = basic_synchronizer.BasicSynchronizer(pid, participants, [2000])

        self.rounds = 3
        self.retries = 4
        self.start()

    def run(self):
        testvar = 3
        self.agent_gvh.create_aw_var('x',int,3)
        self.agent_gvh.create_ar_var('y',int,4)
        self.agent_gvh.put('y',4,self.pid())

        requests, grants = 0,0
        a = BaseMutex(1, [2000])
        self.agent_gvh.mutex_handler.add_mutex(a)
        requested = False
        a.agent_comm_handler = self.agent_comm_handler
        while not self.stopped():
            time.sleep(0.4)
            self.agent_gvh.flush_msgs()
            # print("before",self.agent_gvh.msg_list,self.pid())
            self.agent_comm_handler.handle_msgs()
            # self.agent_gvh.recv_msg_list = []
            # print("after",self.agent_gvh.msg_list,self.pid())

            time.sleep(0.1)

            try:
                if not requested:
                    a.request_mutex()
                    requested = True
                    requests+= 1
                else:
                    if self.agent_gvh.mutex_handler.has_mutex(a.mutex_id):

                        x = self.agent_gvh.get('x')
                        y = self.agent_gvh.get('y', self.pid() - 1)
                        if y is not None:
                            y = y + 1
                        x = x + 1
                        self.agent_gvh.put('x', x)
                        self.agent_gvh.put('y', y, self.pid())
                        grants += 1
                        time.sleep(0.4)
                        a.release_mutex()
                        self.rounds-= 1
                        requested = False

                if self.rounds <= 0 :
                    print("requested",requests,"granted",grants,self.pid())
                    if not self.agent_gvh.is_leader:

                        self.stop()
                        print("the value of test variable is",testvar,"for agent",self.pid())
                        print("the value of x is",self.agent_gvh.get('x'),"for agent", self.pid())
                        print("the value of y is", self.agent_gvh.get('y'),"for agent",self.pid())

                if not self.agent_gvh.is_alive:
                    print("requested",requests,"granted",grants,self.pid())
                    self.stop()

            except CommTimeoutError:
                print("timed out on communication")
                print("requested", requests, "granted", grants)
                self.stop()


a = AgentCreation(0, 3, "", 2000)

