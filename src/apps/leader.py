from src.config.configs import AgentConfig
from src.functionality.base_mutex_handler import BaseMutexHandler
from src.harness.agentThread import AgentThread


class Leader(AgentThread):

    def __init__(self, config):
        super(Leader, self).__init__(config, None)
        self.start()

    def initialize_vars(self):
        self.locals['voted'] = False
        self.locals['leader'] = None
        self.create_aw_var('candidate', int, -1)
        self.initialize_lock('voted')

    def loop_body(self):
        if not self.locals['voted']:
            if not self.lock('voted'):
                return
            if self.read_from_shared('candidate', None) < self.pid():
                self.write_to_shared('candidate', None, self.pid())
            else:
                self.locals['leader'] = self.read_from_shared('candidate', None)

            self.locals['voted'] = True
            self.unlock('voted')
            return
        if self.locals['voted']:
            self.locals['leader'] = self.read_from_shared('candidate', None)
            return


plist = [2000, 2001, 2002, 2003, 2004]
bots = 5
c1 = AgentConfig(2, bots, "", 2001, plist, BaseMutexHandler(False, 2))
c2 = AgentConfig(1, bots, "", 2002, plist, BaseMutexHandler(False, 1))
c4 = AgentConfig(4, bots, "", 2004, plist, BaseMutexHandler(False, 4))
c5 = AgentConfig(5, bots, "", 2000, plist, BaseMutexHandler(False, 5))
c3 = AgentConfig(0, bots, "", 2003, plist, BaseMutexHandler(True, 0), is_leader=True)

b, c, d, e, f = Leader(c1), Leader(c2), Leader(c3), Leader(c4), Leader(c5)
