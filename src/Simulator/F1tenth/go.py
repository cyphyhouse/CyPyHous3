#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDriveStamped

class f1tenth:
    def __init__ (self):
        rospy.init_node('f1tenth', anonymous=True)

        self.pub_vel_left_rear_wheel = rospy.Publisher('/racecar/left_rear_wheel_velocity_controller/command', Float64, queue_size=1)
        self.pub_vel_right_rear_wheel = rospy.Publisher('/racecar/right_rear_wheel_velocity_controller/command', Float64, queue_size=1)
        self.pub_vel_left_front_wheel = rospy.Publisher('/racecar/left_front_wheel_velocity_controller/command', Float64, queue_size=1)
        self.pub_vel_right_front_wheel = rospy.Publisher('/racecar/right_front_wheel_velocity_controller/command', Float64, queue_size=1)

        self.pub_pos_left_steering_hinge = rospy.Publisher('/racecar/left_steering_hinge_position_controller/command', Float64, queue_size=1)
        self.pub_pos_right_steering_hinge = rospy.Publisher('/racecar/right_steering_hinge_position_controller/command', Float64, queue_size=1)

        r = rospy.Rate(10)

    
        rospy.on_shutdown(self.shutdown)

        while not rospy.is_shutdown():
            self.pub_vel_left_rear_wheel.publish(10)
            self.pub_vel_right_rear_wheel.publish(10)
            self.pub_vel_left_front_wheel.publish(10)
            self.pub_vel_right_front_wheel.publish(10)
            self.pub_pos_left_steering_hinge.publish(0.1)
            self.pub_pos_right_steering_hinge.publish(0.1)
            r.sleep()

    def shutdown(self):
        rospy.loginfo("Stop Robot")
	    # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.pub_vel_left_rear_wheel.publish(0)
        self.pub_vel_right_rear_wheel.publish(0)
        self.pub_vel_left_front_wheel.publish(0)
        self.pub_vel_right_front_wheel.publish(0)
	    # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        f1tenth()
    except rospy.ROSInterruptException:
        pass
