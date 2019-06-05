'''
from commHandler import CommHandler
from message import  Message
import time

a = CommHandler('127.0.0.1',2003,2004,0)
b = CommHandler('127.0.0.1',2004,2003,1)


a.send(Message(0,0,"hello",time.time()))
'''

import time

# testing whether agent threads are made properly.
from agentThread import AgentThread
from commHandler import CommHandler
from gvh import Gvh
from message import Message


class CommHandlerTestApp(AgentThread):

    def __init__(self, pid: int, num_bots: int, ip: str, sport: int, rport: int):
        """
        :type pid:int
        :type num_bots: int
        """
        super(CommHandlerTestApp, self).__init__(Gvh(pid, num_bots), CommHandler(ip, sport, rport,pid))
        self.start()

    def run(self):
        rounds = 3
        n_round = 0
        while not self.stopped():
            print("executing agent", self.pid)
            self.comm_handler.send(Message(0, 0, "hello", time.time()))
            if n_round >= rounds:
                self.stop()
            n_round += 1


a = CommHandlerTestApp(0,1,'127.0.0.1',3001,3002)
