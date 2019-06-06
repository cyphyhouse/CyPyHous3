import time

import datatypes
from agentThread import AgentThread
from commHandler import CommHandler
from gvh import Gvh
from messageHandler import update_create


class BasicDsmTest(AgentThread):

    def __init__(self, pid: int, num_bots: int, ip: str = '', port: int = 3291):
        super(BasicDsmTest, self).__init__(Gvh(pid, num_bots), CommHandler(ip, port, port, pid))
        self.start()

    def run(self):
        self.agent_gvh.mk_var('aw', datatypes.dtypes.INT, 'x', 0)
        self.agent_gvh.mk_var('aw', datatypes.dtypes.BOOL, 'y' , False)
        rounds = 5
        nrounds = 0

        while not (self.stopped()):
            self.comm_handler.send(update_create(datatypes.dtypes.INT,self.pid, 'x', 2, time.time()))
            self.comm_handler.send(update_create(datatypes.dtypes.BOOL,self.pid, 'y', True, time.time()))
            time.sleep(1)
            nrounds += 1
            if nrounds >= rounds:
                self.stop()

a = BasicDsmTest(0, 2)
