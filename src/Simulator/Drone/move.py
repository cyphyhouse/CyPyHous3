#!/usr/bin/env python3

import rospy
import rospkg
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from logParser import parse

Mode = 0


def main():
    rospy.init_node('Move')

    robot1_msg = ModelState()
    robot1_msg.model_name = 'drone1'

    robot2_msg = ModelState()
    robot2_msg.model_name = 'drone2'

    if Mode == 0:
        move(robot1_msg, (0,0))
        move(robot2_msg, (0,2))
    elif Mode == 1:
        path = parse()
        for point1, point2 in path:
            move(robot1_msg, point1)
            move(robot2_msg, point2)


def move(state_msg, pose):
    rospy.loginfo("Currently on (%f, %f)", pose[0], pose[1])
    state_msg.pose.position.x = pose[0]
    state_msg.pose.position.y = pose[1]
    state_msg.pose.position.z = 0.0
    state_msg.pose.orientation.x = 0
    state_msg.pose.orientation.y = 0
    state_msg.pose.orientation.z = 0
    state_msg.pose.orientation.w = 0

    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state(state_msg)
    except rospy.ServiceException as e:
        rospy.loginfo("Service call failed with %s", e)


if __name__ == '__main__':
    try:
        main()
    except:
        rospy.loginfo("User pressed  Ctrl-C, quit!")