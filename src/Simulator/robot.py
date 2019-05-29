#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2, sqrt

'''
    The robot class used in Gazebo
'''
class Robot():
    '''
        This function construct a robot initialized to (0,0).
        Args: index - index of current robot
        Return: None
    '''
    def __init__(self, number):
        # Robot's position and orientation inforamtion
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self.goal = Point()

        # Set up subscriber and publisher
        my_number = "/robot" + str(number)
        # Subscribing for position
        self.sub = rospy.Subscriber(my_number+"/ground_truth/state", Odometry, self.newPos)
        # Publish to drive the robot
        self.pub = rospy.Publisher(my_number+"/cmd_vel_mux/input/navi", Twist, queue_size=10)

    '''
        This function is used for subscription to new odometry messages  
        Args: msg - nav_msg for pose and quaternion
        Returns: None
    '''
    def newPos(self, msg):
        # Update pose
        self._x = msg.pose.pose.position.x 
        self._y = msg.pose.pose.position.y 

        # Update orientations
        quat = msg.pose.pose.orientation
        (_, _, self._theta) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

