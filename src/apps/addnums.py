from src.harness.agentThread import AgentThread
import time

class AddNums(AgentThread):
    """
    test class to test that agent thread objects are created
    and fields are accessed safely and correctly .
    """

    def __init__(self, agentconfig):
        super(AddNums, self).__init__(agentconfig, None)
        self.start()

    def initialize_vars(self):
        self.locals['added'] = False
        self.locals['finalsum'] = None
        self.create_ar_var('sum', int, 0)
        self.create_ar_var('numadded', int, 0)
        self.locals['numadded'] = 0
        self.initialize_lock('adding')

    def loop_body(self):
        time.sleep(0.1)
        self.locals['numadded'] = 0
        for pid in range(self.num_agents()):
            self.locals['numadded'] += self.read_from_shared('numadded', pid)
        print("numadded for agent",self.pid()," is ",self.read_from_shared('numadded',None),"on round", self.agent_gvh.round_num)
        if not self.locals['added']:
            if not self.lock('adding'):
                return
            self.write_to_shared('sum', self.pid(), self.read_from_shared('sum', self.pid()) + self.pid() * 2)
            self.write_to_shared('numadded', self.pid(), 1)
            self.locals['added'] = True
            self.unlock('adding')
            return
        if self.locals['numadded'] == self.num_agents():
            self.locals['finalsum'] = self.read_from_shared('sum', self.pid())
            print("final sum is", self.locals['finalsum'],"\n")
            self.trystop()
            return


