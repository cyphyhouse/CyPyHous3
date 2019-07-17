# unittests for agent thread basics without actual behavior

import unittest

from src.harness.agentThread import AgentThread
from src.harness.comm_handler import CommHandler
from src.harness.gvh import Gvh


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
        agent_comm_handler = CommHandler(receiver_ip, r_port, agent_gvh, 1 )
        #agent_comm_handler.agent_gvh = agent_gvh
        super(AgentCreation, self).__init__(agent_gvh, agent_comm_handler)
        self.start()

    def run(self):
        pass


class AgentCreationTest(unittest.TestCase):

    def setUp(self):
        """
        setup. may be used in other tests.
        :return:
        """
        self.testClass = AgentCreation(0, 10, 'localhost', 2000)

    def test_basic_agent_thread_methods(self):
        """
        - test for the gvh object of the agent thread
        - test for the communication handler of the agent thread.
        - testing setter methods for objects related to agent thread.


        :return:
        """

        self.assertEqual(self.testClass.agent_gvh.pid, 0)
        self.assertEqual(self.testClass.participants(), 10)
        self.assertEqual(self.testClass.receiver_port(), 2000)
        self.testClass.agent_comm_handler.ip = 'test'
        self.assertEqual(self.testClass.receiver_ip(), 'test')
        self.testClass.agent_gvh.pid = 1
        self.assertEqual(self.testClass.pid(), 1)
        self.testClass.agent_gvh.participants = 9
        self.assertEqual(self.testClass.participants(), 9)


if __name__ == "__main__":
    unittest.main()
