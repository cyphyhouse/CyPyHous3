import time
from agentThread import AgentThread
from commHandler import CommHandler
from gvh import Gvh
from messageHandler import update_create


class BasicDsmTest(AgentThread):

    def __init__(self, pid: int, num_bots: int, ip: str = '', port: int = 3291):
        agent_gvh = Gvh(pid, num_bots)
        super(BasicDsmTest, self).__init__(agent_gvh, CommHandler(ip, port, port, pid))
        self.start()

    def run(self):
        self.mk_var('aw', int, 'x', 0)
        self.mk_var('aw', bool, 'y', False)

        rounds = 5
        nrounds = 0

        while not (self.stopped()):
            self.flush_msgs()
            self.request_mutex('x')

            self.put(self.pid, 'x', 2)
            self.put(self.pid, 'y', True)
            self.release_mutex('x')
            print(self.pid, self.agent_gvh.agent_dsm.var_list)

            time.sleep(1)
            nrounds += 1
            if nrounds >= rounds:
                self.stop()



