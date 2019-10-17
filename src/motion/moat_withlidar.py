import numpy as np
import rospy
from src.motion.motionautomaton import MotionAutomaton
from src.motion.pos_types import Pos

from sensor_msgs.msg import LaserScan


class MoatWithLidar(MotionAutomaton):

    def __init__(self, config):
        super(MoatWithLidar, self).__init__(config)
        self.tscan = {}
        self.tpos = {}

        self.__sub_lidar = rospy.Subscriber(config.rospy_node.strip("/waypoint_node")+'/racecar/laser/scan', LaserScan, self._getLidarData)


    def _getPositioning(self, data) -> None:
        quat = data.pose.orientation
        import math
        yaw = math.atan2(2 * (quat.x * quat.y + quat.w * quat.z), pow(quat.w, 2) + pow(quat.x,2) - pow(quat.y,2) - pow(quat.z,2))
        self.position = Pos(np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z, yaw]))
        try:
            cur_time = data.header.stamp.secs + data.header.stamp.nsecs*1e-9
            self.tpos[cur_time] = self.position
        except:
            pass

    def _getLidarData(self, data) -> None:
        tmpList = []
        lidar_msg = data
        cur_angle = lidar_msg.angle_max
        cur_time = lidar_msg.header.stamp.secs + lidar_msg.header.stamp.nsecs*1e-9
        for distance in lidar_msg.ranges:
            cur_angle -= lidar_msg.angle_increment
            # If encounter infinite distance, continue
            # if distance == float('inf'):
            #     continue
            
            tmpList.append((distance, cur_angle))
        self.tscan[cur_time] = tmpList

    def _getReached(self, data) -> None:
        a = str(data).upper()
        if 'TRUE' in a:
            self.reached = True

    def moat_init_action(self):
        super().moat_init_action()
        pass

    def moat_exit_action(self):
        super().moat_exit_action()
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
        import rospy
        for wp in path[:-1]:
            self.goTo(wp, 0)
            rospy.sleep(0.1)
        self.goTo(path[-1], 1)

    def run(self):
        import rospy
        rospy.spin()
