import rospy
from geometry_msgs.msg import PoseStamped


class Quadcopter:

    def __init__(self, bot_name='bot1'):
        self.waypoint_count = 0

        self.pub = rospy.Publisher('Waypoint_'+bot_name, PoseStamped)
        rospy.init_node('quad_wp_node', anonymous=True)

    def goTo(self, dest, wp_type=None):
        if wp_type is not None:
            frame_id = str(wp_type)
        else:
            if self.waypoint_count == 0:
                frame_id = '0'
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


class Car:
    def __init__(self, bot_name='bot1'):

        self.pub = rospy.Publisher('Waypoint_'+bot_name, PoseStamped)
        rospy.init_node('car_wp_node', anonymous=True)

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
