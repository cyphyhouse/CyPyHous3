#!/usr/bin/env python3


import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2, sqrt
from robot import Robot


'''
    This class is used for moving multiple robots towards a specific point with physical controller
'''


class GoTo():
    '''
        Initiates robots
    '''

    def __init__(self):
        rospy.init_node('GoTo_test', anonymous=False)
        self.numberOfRobots = 2
        self.robots = []  # Robot list
        self.complete = []  # Indicator of completion of tasks
        for i in range(self.numberOfRobots):
            self.robots.append(Robot(i + 1))
            self.complete.append(0)

        # What to do if shut down
        rospy.on_shutdown(self.shutdown)

    def goto(self, goals):
        rospy.loginfo("Ready to move. To stop Robot , press CTRL + C")
        rospy.loginfo("Robot1 goes to (%f, %f)", pos1['x'], pos2['y'])
        rospy.loginfo("Robot2 goes to (%f, %f)", pos2['x'], pos2['y'])
        # Set rate
        r = rospy.Rate(10)
        # Twist message
        move_cmd = Twist()

        # Set up goal for each robot
        for i in range(self.numberOfRobots):
            self.robots[i].goal.x = goals[i]['x']
            self.robots[i].goal.y = goals[i]['y']

        # Main while loop for moving the robots
        while not rospy.is_shutdown():
            # Check completion requirement
            if sum(self.complete) == self.numberOfRobots:
                return 1

            for i in range(self.numberOfRobots):
                diff_x = self.robots[i].goal.x - self.robots[i]._x
                diff_y = self.robots[i].goal.y - self.robots[i]._y
                angle_to_goal = atan2(diff_y, diff_x)
                # If distance to goal is small enough, DONE!
                if sqrt(diff_x * diff_x + diff_y * diff_y) < 0.05:
                    self.complete[i] = 1
                # If orientation difference is too big, ROTATE!
                elif abs(angle_to_goal - self.robots[i]._theta) > 0.2:
                    move_cmd.linear.x = 0.0
                    move_cmd.angular.z = 0.3
                # Otherwise, move towards goal
                else:
                    move_cmd.linear.x = 0.5
                    move_cmd.angular.z = 0.0

                # Send Twist message
                self.robots[i].pub.publish(move_cmd)

            # Let robot to go
            r.sleep()

    def shutdown(self):
        rospy.loginfo("Stop Robot")
        # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        for i in range(self.numberOfRobots):
            self.robots[i].pub.publish(Twist())
        # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        navigator = GoTo()

        pos1 = {'x': -5, 'y': 6}
        pos2 = {'x': 5, 'y': 6}
        success = navigator.goto([pos1, pos2])

        if success:
            rospy.loginfo("Yep, we made it!")
        else:
            rospy.loginfo("Something is wrong")

        rospy.sleep(1)
    except:
        rospy.loginfo("User pressed  Ctrl-C, quit!")