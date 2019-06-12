#!/usr/bin/env python3

'''
    Use ros messages to drive drones
    Involves physics

'''

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2, sqrt
from drone import Drone


class GoTo():
    def __init__(self):
        rospy.init_node('Drone_Test', anonymous=False)
        self.numberOfDrones = 2
        self.drones = []
        self.complete = []
        for i in range(self.numberOfDrones):
            self.drones.append(Drone(i+1))
            self.complete.append(0)

        # What to do if shut down
        rospy.on_shutdown(self.shutdown)

    def goto(self, goals):
        rospy.loginfo("Ready to move. To stop Drone , press CTRL + C")
        rospy.loginfo("Drone1 goes to (%f, %f)", pos1['x'], pos2['y'])
        rospy.loginfo("Drone2 goes to (%f, %f)", pos2['x'], pos2['y'])
        r = rospy.Rate(10)
        move_cmd = Twist()

        # Set up goal
        for i in range(self.numberOfDrones):
            self.drones[i].goal.x = goals[i]['x']
            self.drones[i].goal.y = goals[i]['y']
            self.drones[i].goal.z = goals[i]['z']

        while not rospy.is_shutdown():
            if sum(self.complete) == self.numberOfDrones:
                return 1

            for i in range(self.numberOfDrones):
                diff_x = self.drones[i].goal.x - self.drones[i]._x
                diff_y = self.drones[i].goal.y - self.drones[i]._y
                diff_z = self.drones[i].goal.z - self.drones[i]._z

                rospy.loginfo("Drone %d is at (%f, %f, %f)", i, self.drones[i]._x, self.drones[i]._y, self.drones[i]._z)

                if sqrt(diff_x*diff_x + diff_y*diff_y + diff_z*diff_z) < 0.05:
                    self.complete[i] = 1 
                else:
                    if abs(diff_x) > 0.1:
                        if diff_x > 0: move_cmd.linear.x = 0.5
                        else: move_cmd.linear.x = -0.5
                    else:
                        move_cmd.linear.x = 0.0

                    if abs(diff_y) > 0.1:
                        if diff_y > 0: move_cmd.linear.y = 0.5
                        else: move_cmd.linear.y = -0.5
                    else:
                        move_cmd.linear.y = 0.0

                    if abs(diff_z) > 0.1:
                        if diff_z > 0: move_cmd.linear.z = 0.5
                        else: move_cmd.linear.z = 0.0
                    else:
                        move_cmd.linear.z = 0.0
            
                self.drones[i].pub.publish(move_cmd)

            r.sleep()

    def shutdown(self):
        rospy.loginfo("Stop Drones")
        # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot

        for i in range(self.numberOfDrones):
            self.drones[i].pub.publish(Twist())
        # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
            
        rospy.sleep(1)

if __name__ == '__main__': 
    try:
        navigator = GoTo()

        pos1 = {'x': 5, 'y': 7, 'z': 10}
        pos2 = {'x': -5, 'y': 7, 'z': 10}
        success = navigator.goto([pos1, pos2])

        if success:
            rospy.loginfo("Yep, we made it!")
        else:
            rospy.loginfo("Something is wrong")
            
        rospy.sleep(1)
    except rospy.ROSInterruptException:
        rospy.loginfo("User pressed  Ctrl-C, quit!")