from src.config.configs import AgentConfig
from src.harness.agentThread import AgentThread
from src.functionality.comm_funcs import send
from src.harness.message_handler import test_mesg_create
import time
from src.functionality.base_mutex_handler import BaseMutexHandler

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

    def loop_body(self):
        time.sleep(0.01)
        if self.locals['seqnum'] <= 10:
            msg = test_mesg_create(self.locals['seqnum'], self.pid())
            if not self.agent_gvh.port_list == []:
                for port in self.agent_gvh.port_list:
                    send(msg, "<broadcast>", port)
            else:
                send(msg, "<broadcast>", self.agent_gvh.rport)



        else:
            if self.locals['seqnum'] == 20:
                self.stop()

        self.locals['seqnum'] += 1


bots = 3

a1 = AgentConfig(0, bots, "", rport=2001, plist=[2001,2002,2003], mh=BaseMutexHandler, is_leader=False, mhargs=[False,0])
a2 = AgentConfig(1, bots, "", rport=2002, plist=[2001,2002,2003], mh=BaseMutexHandler, is_leader=True, mhargs=[True,1])
a3 = AgentConfig(2, bots, "", rport=2003, plist=[2001,2002,2003], mh=BaseMutexHandler, is_leader=False, mhargs=[False,2])

#app1 = TestSndRecv(a1)
app2 = TestSndRecv(a2)
#app3 = TestSndRecv(a3)