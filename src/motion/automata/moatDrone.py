#  Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

import time
import typing

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped

import src.config.configs as configs
import src.datatypes.motion.pos as pos
import src.motion.abstract.motionAutomaton as motionAutomaton


class MoatDrone(motionAutomaton.MotionAutomaton):

    def __init__(self, config: configs.MoatConfig):
        """
        :param config: motion automaton configuration object
        :type config: MoatConfig
        """
        super(MoatDrone, self).__init__(config)

    def _get_positioning(self, data) -> None:
        """
        callback function to get positioning
        """
        self.position = pos.Pos(np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z]))

    def _get_reached(self, data) -> None:
        """
        callback function to determine whether robot reached way point
        """
        a = str(data).upper()
        if 'TRUE' in a:
            self.reached = True

    def takeoff(self) -> None:
        """
        takeoff action for drone
        """
        print("taking off")
        takeoff = pos.Pos(np.array([self.position.x, self.position.y, 1.0]))
        self.go_to(takeoff)

    def land(self) -> None:
        """
        landing action for drone
        """
        print("landing")
        landing = pos.Pos(np.array([self.position.x, self.position.y, 0.0]))
        self.go_to(landing)

    def moat_init_action(self) -> None:
        """
        init action for drone : takeoff
        """
        self.takeoff()
        while not self.reached:
            time.sleep(0.1)

    def moat_exit_action(self):
        """
        exit action for drone : landing
        TODO: maybe incorporate call to best here?
        """
        self.land()

    def go_to(self, dest: pos.Pos, wp_type: typing.Union[int, None] = None) -> None:
        """
        go to destination way point

        :param dest: destination way point
        :type dest: Pos

        :param wp_type: way point type (intermediate or final)
        :type wp_type: int or None.

        """
        if wp_type is not None:
            frame_id = str(wp_type)
        else:
            frame_id = '1'

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
        rospy.spin()
