from agentThread import AgentThread
from gvh import Gvh
import time
import rrt_star
import motionAutomaton
import rospy
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String

class BasicFollowApp(AgentThread):

    def __init__(self, pid: int, num_bots: int):
        agent_gvh = Gvh(pid, num_bots)
        agent_gvh.moat = motionAutomaton.MotionAutomaton(pid, 'f1car',10 )
        super(BasicFollowApp, self).__init__(agent_gvh,None)
        self.start()

    def run(self):
        #local int tries = 1,
        # pos dest1 = new pos(0.,0.,1)
        # pos land = new pos(0.,0.,0.)

        tries = 1


        dest1 = Pose()
        dest1.position.x, dest1.position.y, dest1.position.z = 2., 1., 0.
        dest2 = Pose()
        dest2.position.x, dest2.position.y, dest2.position.z = -1., -1., 0.
        dest3 = Pose()
        dest3.position.x, dest3.position.y, dest3.position.z = 1., 2., 0.

        #dest1 = vec(0.0,1.0,0.0)
        #dest2 = vec(1.0,1.0,0.0)
        while not self.stopped():




            # event 1 : pre (tries == 1) .
            # eff : Motion.target = dest1
            # tries = 2

            #event 2: pre(tries == 2)
            # eff : Motion.target = dest2
            # tries = 3
            # stop


            if tries == 1:
                path1 = rrt_star.RRT([self.agent_gvh.moat.position.position.x,
                                      self.agent_gvh.moat.position.position.y], [dest1.position.x, dest1.position.y]).Planning()
                self.agent_gvh.moat.follow_path(path1)
                #self.agent_gvh.moat.follow_path([dest1])
                time.sleep(5)
                tries = 2
                continue
            if tries == 2:
                path2 = rrt_star.RRT([self.agent_gvh.moat.position.position.x,
                                      self.agent_gvh.moat.position.position.y], [dest2.position.x, dest2.position.y]).Planning()
                self.agent_gvh.moat.follow_path(path2)
                #self.agent_gvh.moat.follow_path([dest2])

                tries = 3
                time.sleep(5)
                continue
            if tries == 3:
                path3 = rrt_star.RRT([self.agent_gvh.moat.position.position.x,
                                      self.agent_gvh.moat.position.position.y],
                                     [dest3.position.x, dest3.position.y]).Planning()
                self.agent_gvh.moat.follow_path(path3)
                # self.agent_gvh.moat.follow_path([dest2])
                tries = 4
                time.sleep(5)
                self.stop()
                continue

BasicFollowApp(1,1)


