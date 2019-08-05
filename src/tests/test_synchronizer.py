import unittest

from src.config.configs import AgentConfig
from src.functionality.base_mutex_handler import BaseMutexHandler
from src.functionality.basic_synchronizer import BasicSynchronizer
from src.harness.agentThread import AgentThread


class BasicSync(AgentThread):
    """
    test class to test that agent thread objects are created
    and fields are accessed safely and correctly .
    """

    def __init__(self, agentconfig):
        super(BasicSync, self).__init__(agentconfig, None)
        self.agent_gvh.synchronizer = BasicSynchronizer(agentconfig)
        self.start()

    def initialize_vars(self):
        self.agent_gvh.synchronizer.send_sync_message()
        self.locals['sync_var'] = 0

    def loop_body(self):
        self.agent_gvh.synchronizer.synchronize_wait()
        if self.locals['sync_var'] < 3:
            self.locals['sync_var'] += 1
            if self.agent_gvh.synchronizer.is_synced:
                print("round", self.agent_gvh.synchronizer.round_num, self.pid())
        else:
            return
        self.agent_gvh.synchronizer.send_sync_message()



class TestSync(unittest.TestCase):
    """
    test class to test that agent thread objects are created
    and fields are accessed safely and correctly .
    """

    def setUp(self):
        plist = [2000, 2001, 2002, 2003, 2004]
        bots = 5
        c1 = AgentConfig(2, bots, "", 2001, plist, BaseMutexHandler, mhargs=[False, 2])
        c2 = AgentConfig(1, bots, "", 2002, plist, BaseMutexHandler, mhargs=[False, 1])
        c4 = AgentConfig(4, bots, "", 2004, plist, BaseMutexHandler, mhargs=[False, 4])
        c5 = AgentConfig(5, bots, "", 2000, plist, BaseMutexHandler, mhargs=[False, 5])
        c3 = AgentConfig(0, bots, "", 2003, plist, BaseMutexHandler, mhargs=[True, 0], is_leader=True)

        self.b, self.c, self.d, self.e, self.f = BasicSync(c1), BasicSync(c2), BasicSync(c3), BasicSync(c4), BasicSync(c5)


    def test_sync(self):
        pass

    def tearDown(self):
        self.b.agent_comm_handler.receiver_socket.close()
        self.c.agent_comm_handler.receiver_socket.close()
        self.d.agent_comm_handler.receiver_socket.close()
        self.e.agent_comm_handler.receiver_socket.close()
        self.f.agent_comm_handler.receiver_socket.close()


if __name__ == '__main__':
    unittest.main()

'''
plist = [2000, 2001, 2002, 2003, 2004]
bots = 5
c1 = AgentConfig(2, bots, "", 2001, plist, BaseMutexHandler(False, 2))
c2 = AgentConfig(1, bots, "", 2002, plist, BaseMutexHandler(False, 1))
c4 = AgentConfig(4, bots, "", 2004, plist, BaseMutexHandler(False, 4))
c5 = AgentConfig(5, bots, "", 2000, plist, BaseMutexHandler(False, 5))
c3 = AgentConfig(0, bots, "", 2003, plist, BaseMutexHandler(True, 0), is_leader=True)

b, c, d, e, f = AddNums(c1), AddNums(c2), AddNums(c3), AddNums(c4), AddNums(c5)
'''
