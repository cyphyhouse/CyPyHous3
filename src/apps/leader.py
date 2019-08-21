from src.harness.agentThread import AgentThread

class Leader(AgentThread):

    def __init__(self, config):
        super(Leader, self).__init__(config, None)
        self.start()

    def initialize_vars(self):
        self.locals['voted'] = False
        self.locals['leader'] = None
        self.create_aw_var('candidate', int, -1)
        self.create_aw_var('numvoted', int, 0)
        self.initialize_lock('voted')

    def loop_body(self):
        if not self.locals['voted']:
            if not self.lock('voted'):
                return
            if self.read_from_shared('candidate', None) < self.pid():
                self.write_to_shared('candidate', None, self.pid())
            else:
                self.locals['leader'] = self.read_from_shared('candidate', None)
            self.write_to_shared('numvoted', None, self.read_from_shared('numvoted', None) + 1)

            self.locals['voted'] = True
            self.unlock('voted')
            return
        if self.locals['voted']:
            self.locals['leader'] = self.read_from_shared('candidate', None)
            if self.read_from_shared('numvoted', None) ==  self.num_agents():
                self.stop()
            return



