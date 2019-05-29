# tests whether the goto function publishes the correct ros messages

import motionautomaton

from geometry_msgs.msg import Pose

quad_m = motionautomaton.Motionautomaton()

dest1 = Pose()
dest1.position.x, dest1.position.y,dest1.position.z = 1.,2.,3.

quad_m.start()

quad_m.goTo(dest1)
