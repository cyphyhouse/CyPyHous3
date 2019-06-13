import time

from agentThread import AgentThread
from commHandler import CommHandler
from gvh import Gvh


class BasicDsmTest1(AgentThread):

    def __init__(self, pid: int, num_bots: int, ip: str = '', rport: int = 3292, sport: int = 3292):
        agent_gvh = Gvh(pid, num_bots)
        super(BasicDsmTest1, self).__init__(agent_gvh, CommHandler(ip, rport, sport, pid))
        self.start()

    def run(self):
        self.mk_var('aw', int, 'x', 0)
        self.mk_var('aw', bool, 'y', False)

        rounds = 10
        nrounds = 0
        requested_mutex = False

        while not (self.stopped()):
            self.flush_msgs()
            if not requested_mutex:
                requested_mutex = True
                self.request_mutex('x')
            else:
                if not self.has_mutex('x'):
                    pass
                else:
                    x = self.agent_gvh.agent_dsm.get('x')
                    print(x, self.pid, nrounds)
                    self.put(self.pid, 'x', x + self.pid)
                    self.release_mutex('x')
                    requested_mutex = False

            time.sleep(1)
            nrounds += 1
            if nrounds >= rounds:
                self.stop()


a = BasicDsmTest1(0, 2, "", 3292, 3292)
