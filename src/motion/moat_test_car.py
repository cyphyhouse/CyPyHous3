import numpy as np

from src.motion.motionautomaton import MotionAutomaton
from src.motion.pos import Pos


class MoatTestCar(MotionAutomaton):

    def __init__(self, config):
        super(MoatTestCar, self).__init__(config)

    def _getPositioning(self, data) -> None:
        print("this function just got called")
        quat = data.pose.orientation
        import math
        yaw = math.atan2(2 * (quat.x * quat.y + quat.w * quat.z), pow(quat.w, 2) + pow(quat.x,2) - pow(quat.y,2) - pow(quat.z,2))
        self.position = Pos(np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z, yaw]))

    def _getReached(self, data) -> None:
        a = str(data).upper()
        if 'TRUE' in a:
            self.reached = True

    def moat_init_action(self):
        pass

    def moat_exit_action(self):
        pass

    def goTo(self, dest: Pos, wp_type: int = None) -> None:
        print("going to point", dest)
        if wp_type is not None:
            frame_id = str(wp_type)
        else:
            frame_id = '1'

        import rospy
        from geometry_msgs.msg import PoseStamped

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = frame_id
        pose.pose = dest.to_pose()

        self.reached = False
        self.waypoint_count += 1
        self.pub.publish(pose)

    def follow_path(self, path: list) -> None:
        for wp in path[:-1]:
            self.goTo(wp, 0)
        self.goTo(path[-1], 1)

    def run(self):
        import rospy
        rospy.spin()
