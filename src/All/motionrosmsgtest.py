# tests whether the goto function publishes the correct ros messages
import time
import motionautomaton

from geometry_msgs.msg import Pose

quad_m = motionautomaton.MotionAutomaton()

dest1 = Pose()
dest1.position.x, dest1.position.y,dest1.position.z = 1.,2.,3.

quad_m.start()
quad_m.goTo(dest1)
time.sleep(1)
quad_m.goTo(dest1)

