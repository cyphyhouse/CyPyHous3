import time

from src.config.configs import AgentConfig
from src.functionality.base_mutex_handler import BaseMutexHandler
from src.harness.agentThread import AgentThread
from src.objects.base_mutex import BaseMutex


class AddNums(AgentThread):
    """
    test class to test that agent thread objects are created
    and fields are accessed safely and correctly .
    """

    def __init__(self, config):
        super(AddNums, self).__init__(config, None)
        self.requestedlock = False
        self.req_num = 0
        self.baselock = None
        self.start()

    def initialize_vars(self):
        self.baselock = BaseMutex(1, self.agent_gvh.port_list)
        self.agent_gvh.mutex_handler.add_mutex(self.baselock)
        self.baselock.agent_comm_handler = self.agent_comm_handler
        self.create_aw_var('sum', int, 0)
        self.create_aw_var('numadded', int, 0)
        self.locals['added'] = False
        self.locals['finalsum'] = 0

    def loop_body(self):

        if not self.locals['added']:
            # lock()
            if not self.requestedlock:
                self.baselock.request_mutex(self.req_num)
                self.requestedlock = True
                self.req_num += 1
                return
            else:
                if not self.agent_gvh.mutex_handler.has_mutex(self.baselock.mutex_id):
                    return
            # print("doing the rest", self.pid())
            self.agent_gvh.put('sum', self.agent_gvh.get('sum') + self.pid() * 2)
            self.agent_gvh.put('numadded', self.agent_gvh.get('numadded') + 1)
            self.locals['added'] = True
            self.baselock.release_mutex()
            # unlock()
            time.sleep(0.1)

            self.requestedlock = False

        elif self.agent_gvh.get('numadded') >= self.agent_gvh.participants:
            self.locals['finalsum'] = self.agent_gvh.get('sum')
            print('final sum for', self.pid(), 'is', self.locals['finalsum'])
            self.stop()


plist = [2000, 2001, 2002, 2003, 2004]
bots = 5
c1 = AgentConfig(2, bots, "", 2001, plist, BaseMutexHandler(False, 2))
c2 = AgentConfig(1, bots, "", 2002, plist, BaseMutexHandler(False, 1))
c4 = AgentConfig(4, bots, "", 2004, plist, BaseMutexHandler(False, 4))
c5 = AgentConfig(5, bots, "", 2000, plist, BaseMutexHandler(False, 5))
c3 = AgentConfig(0, bots, "", 2003, plist, BaseMutexHandler(True, 0), is_leader=True)

b, c, d, e, f = AddNums(c1), AddNums(c2), AddNums(c3), AddNums(c4), AddNums(c5)
