#  Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

import numpy as np

from src.motion.abstract.motionAutomaton import MotionAutomaton
from src.datatypes.motion.pos import Pos


class MoatCar(MotionAutomaton):

    def __init__(self, config):
        super(MoatCar, self).__init__(config)

    def _get_positioning(self, data) -> None:
        quat = data.pose.orientation
        import math
        yaw = math.atan2(2 * (quat.x * quat.y + quat.w * quat.z), pow(quat.w, 2) + pow(quat.x,2) - pow(quat.y,2) - pow(quat.z,2))
        self.position = Pos(np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z, yaw]))

    def _get_reached(self, data) -> None:
        a = str(data).upper()
        if 'TRUE' in a:
            self.reached = True

    def moat_init_action(self):
        pass

    def moat_exit_action(self):
        pass

    def go_to(self, dest: Pos, wp_type: int = None) -> None:
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

        self.__reached = False
        self.__way_point_count += 1
        self.__pub.publish(pose)

    def follow_path(self, path: list) -> None:
        for wp in path[1:-1]:
            self.go_to(wp, 0)
        self.go_to(path[-1], 1)

    def run(self):
        import rospy
        rospy.spin()

