import numpy as np

from src.motion.motionautomaton import MotionAutomaton
from src.motion.pos import Pos


class MoatTestDrone(MotionAutomaton):

    def __init__(self, config):
        super(MoatTestDrone, self).__init__(config)

    def _getPositioning(self, data) -> None:
        self.position = Pos(np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z]))

    def _getReached(self, data) -> None:
        a = str(data).upper()
        if 'TRUE' in a:
            self.reached = True

    def takeoff(self):
        print("taking off")
        takeoff = Pos(np.array([self.position.x, self.position.y, 1.0]))
        self.goTo(takeoff)

    def land(self):
        print("landing")
        landing = Pos(np.array([self.position.x, self.position.y, 0.0]))
        self.goTo(landing)

    def moat_init_action(self):
        self.takeoff()

    def moat_exit_action(self):
        # TODO: maybe incorporate call to best here?
        self.land()
        pass

    def goTo(self, dest: Pos, wp_type: int = None) -> None:
        print("going to point", dest)
        if wp_type is not None:
            frame_id = str(wp_type)
        else:
            #if self.waypoint_count == 0:
            #    frame_id = '0'
            #elif dest.z <= 0:
            #    frame_id = '1'
            #else:
            frame_id = '1'

        import rospy
        from geometry_msgs.msg import PoseStamped

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = frame_id
        pose.pose = dest.to_pose()

        self.reached = False

        self.waypoint_count += 1
        print(self.pub)
        self.pub.publish(pose)

    def follow_path(self, path: list) -> None:
        for wp in path[:-1]:
            self.goTo(wp, 0)
        self.goTo(path[-1], 1)

    def run(self):
        import rospy
        rospy.spin()
