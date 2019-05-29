from agentThread import AgentThread
from gvh import Gvh


class TestApp(AgentThread):

    def __init__(self, pid, numbots):
        super(TestApp, self).__init__(Gvh(pid, numbots))
        self.start()

    def run(self):
        rounds: int = 3
        nround: int = 0
        while not self.stopped():
            print("executing agent", self.pid)
            if nround >= rounds:
                self.stopped()
            nround += 1


for i in range(0, 3):
    a = TestApp(i, 3)
