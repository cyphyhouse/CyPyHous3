from socketserver import DatagramRequestHandler, ThreadingUDPServer
from threading import Thread
import time
import unittest

from src.config.configs import AgentConfig
from src.functionality.base_mutex_handler import BaseMutexHandler
from src.harness.agentThread import AgentThread


class TestAR(AgentThread):
    """
    test class to test that agent thread objects are created
    and fields are accessed safely and correctly .
    """

    def __init__(self, agentconfig):
        super(TestAR, self).__init__(agentconfig, None)
        self.start()

    def initialize_vars(self):
        self.locals['added'] = False
        self.locals['finalsum'] = None
        self.create_aw_var('sum', int, 0)
        if self.pid() == self.num_agents() - 1:
            self.locals['neighbor'] = 0
        else:
            self.locals['neighbor'] = self.pid() + 1
        self.create_ar_var('sum1', int, 0)
        self.create_ar_var('mypid', int, self.pid())
        self.create_aw_var('numadded', int, 0)
        self.initialize_lock('adding')

    def loop_body(self):
        if not self.locals['added']:
            if not self.lock('adding'):
                return

            self.write_to_shared('sum', None, self.read_from_shared('sum', None) + self.read_from_shared('mypid', self.locals['neighbor']))
            self.write_to_shared('sum1', self.pid(), self.read_from_shared('sum',None))
            self.write_to_shared('numadded', None, self.read_from_shared('numadded', None) + 1)
            self.locals['added'] = True
            self.unlock('adding')
            return
        if self.read_from_shared('numadded', None) == self.num_agents():
            self.locals['finalsum'] = self.read_from_shared('sum', None)
            self.stop()
            return


class AllReadTest(unittest.TestCase):
    """
    test class to test that agent thread objects are created
    and fields are accessed safely and correctly .
    """

    def setUp(self):
        plist = None
        port = 2002
        bots = 5

        c1 = AgentConfig(2, bots, "127.0.1.2", port, plist, BaseMutexHandler, mhargs=[False, 2])
        c2 = AgentConfig(1, bots, "127.0.1.1", port, plist, BaseMutexHandler, mhargs=[False, 1])
        c4 = AgentConfig(4, bots, "127.0.1.4", port, plist, BaseMutexHandler, mhargs=[False, 4])
        c5 = AgentConfig(3, bots, "127.0.1.3", port, plist, BaseMutexHandler, mhargs=[False, 3])
        c3 = AgentConfig(0, bots, "127.0.1.0", port, plist, BaseMutexHandler, mhargs=[True, 0], is_leader=True)

        class SimUDPBCastHandler(DatagramRequestHandler):
            def handle(self):
                data = self.request[0].strip()
                socket = self.request[1]
                for addr in ["127.0.1.0", "127.0.1.1", "127.0.1.2", "127.0.1.3", "127.0.1.4"]:
                    socket.sendto(data, (addr, port))

        self.server = ThreadingUDPServer((AgentConfig.BROADCAST_ADDR, port), SimUDPBCastHandler)
        self.server_thread = Thread(target=self.server.serve_forever)
        self.server_thread.start()

        self.b, self.c, self.d, self.e, self.f = TestAR(c1), TestAR(c2), TestAR(c3), TestAR(c4), TestAR(c5)

    def test_write_to_shared(self):
        time.sleep(8)
        self.finalsum = self.b.locals['finalsum']
        # self.numvoted = self.b.read_from_shared('numvoted', None)
        print(self.b.agent_gvh.dsm)
        self.assertEqual(self.finalsum, 10)

    def tearDown(self):
        self.b.agent_comm_handler.receiver_socket.close()
        self.c.agent_comm_handler.receiver_socket.close()
        self.d.agent_comm_handler.receiver_socket.close()
        self.e.agent_comm_handler.receiver_socket.close()
        self.f.agent_comm_handler.receiver_socket.close()

        self.server.shutdown()
        self.server_thread.join()
        self.server.server_close()


if __name__ == '__main__':
    unittest.main()
