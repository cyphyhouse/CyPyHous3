import time

import datatypes
from agentThread import AgentThread
from commHandler import CommHandler
from gvh import Gvh
from messageHandler import update_create


class BasicDsmTest(AgentThread):

    def __init__(self, pid: int, num_bots: int, ip: str = '127.0.0.1', port: int = 3291):
        super(BasicDsmTest, self).__init__(Gvh(pid, num_bots), CommHandler(ip, port, port, pid))
        self.start()

    def run(self):
        self.agent_gvh.mk_var('aw', datatypes.dtypes.INT, 'x', 0)
        self.agent_gvh.mk_var('aw', datatypes.dtypes.BOOL, 'y' , False)
        rounds = 5
        nrounds = 0

        while not (self.stopped()):
            print("running", self.pid)
            self.comm_handler.send(update_create(datatypes.dtypes.INT,self.pid, 'x', 2, time.time()))
            self.comm_handler.send(update_create(datatypes.dtypes.BOOL,self.pid, 'y', True, time.time()))

            time.sleep(1)
            print(self.pid," has ",self.agent_gvh.agent_dsm.var_list)
            nrounds += 1
            if nrounds >= rounds:
                self.stop()




a = BasicDsmTest(1, 2)
