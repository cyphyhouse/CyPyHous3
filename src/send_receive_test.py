import time
import time

from agentThread import AgentThread, send
from comm_handler import CommHandler
from gvh import Gvh
from message import Message


class SndRecv(AgentThread):
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
        super(SndRecv, self).__init__(agent_gvh, agent_comm_handler)
        self.start()

    def run(self):
        send(Message(1, 2, "three", 4.0), '', self.receiver_port())
        time.sleep(1)
        send(Message(2, 3, "four", 5.0), '', self.receiver_port())
        self.stop()


a = SndRecv(0, 1, '', 2000)
