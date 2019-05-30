import time

from geometry_msgs.msg import Pose

from agentThread import AgentThread
from gvh import Gvh


class BasicFollowApp(AgentThread):

    def __init__(self, pid: int, num_bots: int):
        super(BasicFollowApp, self).__init__(Gvh(pid, num_bots))
        self.start()

    def run(self):
        dest1 = Pose()
        dest1.position.x, dest1.position.y, dest1.position.z = 1., 2., 3.

        time.sleep(0.1)
        self.agent_gvh.moat.goTo(dest1)
