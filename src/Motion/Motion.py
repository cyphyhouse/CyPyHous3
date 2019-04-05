import rospy
from geometry_msgs.msg import PoseStamped, Pose
from threading import Thread
import math
import numpy as np


class Quadcopter(Thread):

    def __init__(self, bot_num=1, bot_name='cyphyhousecopter'):
        Thread.__init__(self)
        self.waypoint_count = 0
        self.position = Pose()
        self.heading = 0

        rospy.init_node('quad_wp_node', anonymous=True)
        self.pub = rospy.Publisher('Waypoint_bot'+str(bot_num), PoseStamped, queue_size=1)
        self.sub_vicon = rospy.Subscriber('/vrpn_client_node/'+bot_name+'/pose', PoseStamped, self._getVicon, queue_size=1)

    def _getVicon(self, data):
        self.position = data.pose.position
        quat = data.pose.orientation
        self.heading = math.atan2(2 * (quat.x * quat.y + quat.w * quat.z), quat.w**2 + quat.x**2 - quat.y**2 - quat.z**2)

    def goTo(self, dest, wp_type=None):
        if wp_type is not None:
            frame_id = str(wp_type)
        else:
            if self.waypoint_count == 0:
                frame_id = '0'
            elif dest[2] <= 0:
                frame_id = '2'
            else:
                frame_id = '1'

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = frame_id
        pose.pose.position.x = dest[0]
        pose.pose.position.y = dest[1]
        pose.pose.position.z = dest[2]

        self.waypoint_count += 1
        self.pub.publish(pose)

    def getPosition(self):
        return np.array([self.position.x, self.position.y, self.position.z, self.heading])

    def run(self):
        rospy.spin()


class Car(thread):
    def __init__(self, bot_num=1, bot_name='f1_car'):
        Thread.__init__(self)
        self.position = Pose()

        rospy.init_node('car_wp_node', anonymous=True)
        self.pub = rospy.Publisher('Waypoint_bot'+str(bot_num), PoseStamped)
        self.sub_vicon = rospy.Subscriber('/vrpn_client_node/' + bot_name + '/pose', PoseStamped, self._getVicon, queue_size=1)

    def _getVicon(self, data):
        self.position = data.pose.position

    def goTo(self, dest, wp_type=None):
        if wp_type is not None:
            frame_id = str(wp_type)
        else:
            frame_id = '0'

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = frame_id
        pose.pose.position.x = dest[0]
        pose.pose.position.y = dest[1]
        pose.pose.position.z = dest[2]

        self.pub.publish(pose)

    def getPosition(self):
        return np.array([self.position.x, self.position.y, self.position.z, self.heading])

    def run(self):
        rospy.spin()
