import time

import basic_synchronizer
from agentThread import AgentThread, send
from comm_handler import CommHandler, CommTimeoutError
from gvh import Gvh
from message_handler import round_update_create


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
        :param s_port:
        :param r_port:
        """
        agent_gvh = Gvh(pid, participants)
        agent_comm_handler = CommHandler(receiver_ip, r_port)
        super(AgentCreation, self).__init__(agent_gvh, agent_comm_handler)
        self.agent_gvh.synchronizer = basic_synchronizer.BasicSynchronizer(pid, participants, [2000, 2001])
        self.agent_comm_handler.agent_gvh = self.agent_gvh
        self.start()

    def run(self):
        while not self.stopped():
            started = False
            if not started :
                msg = round_update_create(self.pid(), self.agent_gvh.synchronizer.round_num, time.time())

                for port in self.agent_gvh.synchronizer.ip_port_list:
                    send(msg,"192.168.1.255",port)
                self.agent_gvh.add_msg(msg)
                self.agent_gvh.flush_msgs()
                time.sleep(1)
            try:
                self.agent_comm_handler.handle_msgs()
                self.agent_gvh.synchronizer.synchronize()
                started = True
                print("here")
            except basic_synchronizer.RoundSyncError:
                print("waiting for everyone to start")
                time.sleep(0.1)
            else:
                if self.agent_gvh.synchronizer.round_num < 10:
                    msg = round_update_create(self.pid(), self.agent_gvh.synchronizer.round_num, time.time())
                    for port in self.agent_gvh.synchronizer.ip_port_list:
                        send(msg,"192.168.1.255",port)
                    self.agent_gvh.add_msg(msg)
                    self.agent_gvh.flush_msgs()
                    self.agent_comm_handler.handle_msgs()
                    try:
                        self.agent_gvh.synchronizer.synchronize()
                        print("round",self.agent_gvh.synchronizer.round_num,"on agent",self.pid())

                    except basic_synchronizer.RoundSyncError:
                        print("waiting to sync")
                        time.sleep(0.01)
                        continue

                    if self.agent_gvh.is_alive :
                        continue
                    else:
                        self.stop()
                else:
                    self.stop()

b = AgentCreation(1, 2, "", 2001)
a = AgentCreation(0, 2, "", 2000)
