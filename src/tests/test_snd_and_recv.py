import time

from src.config.configs import AgentConfig
from src.functionality.base_mutex_handler import BaseMutexHandler
from src.harness.agentThread import AgentThread
from src.functionality.comm_funcs import send
from src.harness.message_handler import test_mesg_create


class TestSndRecv(AgentThread):
    """
    test class to test that agent thread objects are created
    and fields are accessed safely and correctly .
    """

    def __init__(self, agentconfig):
        super(TestSndRecv, self).__init__(agentconfig, None)
        self.start()

    def initialize_vars(self):
        self.locals['seqnum'] = 0
        self.create_aw_var('seqnum', int, 0)
        self.initialize_lock('adding')
        self.locals['loopno'] = 0

    def loop_body(self):
        msg = test_mesg_create(self.locals['seqnum'], self.pid())
        if self.pid() == 0:
            if not self.agent_gvh.port_list == []:
                for port in self.agent_gvh.port_list:
                    send(msg, "<broadcast>", port)
            else:
                send(msg, "<broadcast>", self.agent_gvh.rport)

            self.locals['seqnum'] += 1

            if self.locals['seqnum'] == 20:
                self.stop()

