from agentThread import AgentThread
from gvh import Gvh
import time
from rrt_star import vec

class BasicFollowApp(AgentThread):

    def __init__(self, pid: int, num_bots: int):
        super(BasicFollowApp, self).__init__(Gvh(pid, num_bots))
        self.start()

    def run(self):
        #local int tries = 1,
        # pos dest1 = new pos(0.,0.,1)
        # pos land = new pos(0.,0.,0.)

        tries = 1

        #dest1 = Pose()
        #dest1.position.x, dest1.position.y, dest1.position.z = 0., 1., 0.
        #land = Pose()
        #land.position.x, land.position.y, land.position.z = 0., 0., 0.

        dest1 = vec(0.0,1.0,0.0)
        dest2 = vec(0.0,1.0,1.0)
        while not self.stopped():
            time.sleep(2.0)



            # event 1 : pre (tries == 1) .
            # eff : Motion.target = dest1
            # tries = 2

            #event 2: pre(tries == 2)
            # eff : Motion.target = dest2
            # tries = 3
            # stop


            if tries == 1:
                self.agent_gvh.moat.follow_path([dest1])
                tries = 2
                continue
            if tries == 2:
                self.agent_gvh.moat.follow_path([dest2])
                tries = 3
                self.stop()
                continue



