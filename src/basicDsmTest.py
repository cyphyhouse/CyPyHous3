import time

from agentThread import AgentThread
from gvh import Gvh
from commHandler import CommHandler


class BasicDsmTest(AgentThread):

    def __init__(self, pid: int, num_bots: int, ip: str = '127.0.0.1', port: int = 3291):

        super(BasicDsmTest, self).__init__(Gvh(pid, num_bots, ip, port))
        self.start()


def run(self):
    pass
