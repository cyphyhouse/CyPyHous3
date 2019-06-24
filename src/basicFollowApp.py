from agentThread import AgentThread
from geometry_msgs.msg import Pose
from gvh import Gvh
import time


class BasicFollowApp(AgentThread):

    def __init__(self, pid: int, num_bots: int):
        super(BasicFollowApp, self).__init__(Gvh(pid, num_bots))
        self.start()

    def run(self):
        #local int tries = 1,
        # pos dest1 = new pos(0.,0.,1)
        # pos land = new pos(0.,0.,0.)

        tries = 1

        dest1 = Pose()
        dest1.position.x, dest1.position.y, dest1.position.z = 0., 0., 1.
        land = Pose()
        land.position.x, land.position.y, land.position.z = 0., 0., 0.
        while not self.stopped():
            time.sleep(1.0)



            # event 1 : pre (tries == 1) .
            # eff : Motion.target = dest1
            # tries = 2

            #event 2: pre(tries == 2)
            # eff : Motion.target = dest2
            # tries = 3
            # stop


            if tries == 1:
                self.agent_gvh.moat.goTo(dest1)
                tries = 2
                continue
            if tries == 2:
                self.agent_gvh.moat.goTo(land)
                tries = 3
                self.stop()
                continue




