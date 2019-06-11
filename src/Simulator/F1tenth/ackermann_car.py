#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PointStamped, Twist, Pose
from std_msgs.msg import Float64
from math import atan2, sqrt
from ackermann_msgs.msg import AckermannDriveStamped

class Car():
    def __init__(self):
        rospy.init_node("Ackermann", anonymous=False)

        self.throttle = 0.0
        self.steer = 0.0

        # Set up subscriber and publisher
        self.pub_vel_left_rear_wheel = rospy.Publisher('/racecar/left_rear_wheel_velocity_controller/command', Float64, queue_size=1)
        self.pub_vel_right_rear_wheel = rospy.Publisher('/racecar/right_rear_wheel_velocity_controller/command', Float64, queue_size=1)
        self.pub_vel_left_front_wheel = rospy.Publisher('/racecar/left_front_wheel_velocity_controller/command', Float64, queue_size=1)
        self.pub_vel_right_front_wheel = rospy.Publisher('/racecar/right_front_wheel_velocity_controller/command', Float64, queue_size=1)

        self.pub_pos_left_steering_hinge = rospy.Publisher('/racecar/left_steering_hinge_position_controller/command', Float64, queue_size=1)
        self.pub_pos_right_steering_hinge = rospy.Publisher('/racecar/right_steering_hinge_position_controller/command', Float64, queue_size=1)

        rospy.loginfo("Send goal!")
        self.pub_goal = rospy.Publisher("/car_goal/goal", PointStamped, queue_size=1)
        goal = PointStamped()
        goal.point.x = 10
        goal.point.y = -10
        self.pub_goal.publish(goal)

        rospy.loginfo("Subscribe ackermann message!")
        self.ackermann = rospy.Subscriber("/ackermann_cmd", AckermannDriveStamped, self.set_throttle)

        rospy.spin()

    def set_throttle(self, data):
        rospy.loginfo("Command recieved!")
        self.throttle = data.drive.speed/0.1
        self.steer = data.drive.steering_angle
        rospy.loginfo("Set throttle: %f, set steer: %f. ", self.throttle, self.steer)
        self.pub_pos_right_steering_hinge.publish(self.steer)
        self.pub_pos_left_steering_hinge.publish(self.steer)
        self.pub_vel_left_rear_wheel.publish(self.throttle)
        self.pub_vel_right_rear_wheel.publish(self.throttle)
        self.pub_vel_left_front_wheel.publish(self.throttle)
        self.pub_vel_right_front_wheel.publish(self.throttle)

if __name__ == '__main__':
    try:
        Car()

    except rospy.ROSInterruptException:
        rospy.loginfo("User pressed  Ctrl-C, quit!")


