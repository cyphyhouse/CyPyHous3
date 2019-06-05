# testing whether agent threads are made properly.
from agentThread import AgentThread
from gvh import Gvh


class TestApp(AgentThread):

    def __init__(self, pid: int, num_bots: int):
        """
        :type pid:int
        :type num_bots: int
        """
        super(TestApp, self).__init__(Gvh(pid, num_bots))
        self.start()

    def run(self):
        rounds: int = 3
        n_round: int = 0
        while not self.stopped():
            print("executing agent", self.pid)
            if n_round >= rounds:
                self.stop()
            n_round += 1


for i in range(0, 3):
    a = TestApp(i, 3)
