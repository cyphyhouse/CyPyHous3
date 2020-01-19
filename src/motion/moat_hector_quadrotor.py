"""
Motion Automaton implementation for Hector Quadrotor ROS package
http://wiki.ros.org/hector_quadrotor
"""

import numpy as np

from actionlib import SimpleActionClient, GoalStatus
from geometry_msgs.msg import PoseStamped
from hector_uav_msgs import msg as hq_msg
import rospy

from src.motion.motionautomaton import MotionAutomaton
from src.motion.pos_types import Pos


class MoatHectorQuadrotor(MotionAutomaton):

    def __init__(self, config):
        act_prefix = config.bot_name + "/action/"  # FIXME better way to find namespace for actions
        # XXX Takeoff has to be assigned before super().__init__
        #  because moat_init_action is called and uses it
        self.__takeoff_act = act_prefix + "takeoff"

        super(MoatHectorQuadrotor, self).__init__(config)

        self.__landing_act = act_prefix + "landing"
        self.__pose_client = SimpleActionClient(act_prefix + "pose", hq_msg.PoseAction)

        # TODO hector_quadrotor package already provides necessary actions,
        #  we may unregister unnecessary publishers and subscribers to save bandwidth

    def _getPositioning(self, data) -> None:
        self.position = Pos(np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z]))

    def _getReached(self, data) -> None:
        if data.upper() == "TRUE":
            self.reached = True

    def moat_init_action(self):
        super().moat_init_action()

        client = SimpleActionClient(self.__takeoff_act, hq_msg.TakeoffAction)
        client.wait_for_server()  # TODO Set Timeout?
        goal = hq_msg.TakeoffGoal()
        print("taking off")
        result = client.send_goal_and_wait(goal)  # TODO Set Timeout?
        # TODO Check result
        print("take off successful")

    def moat_exit_action(self):
        client = SimpleActionClient(self.__landing_act, hq_msg.LandingAction)
        client.wait_for_server()  # TODO Set Timeout?
        goal = hq_msg.LandingGoal()

        print("landing")
        result = client.send_goal_and_wait(goal)  # TODO Set Timeout?
        # TODO check result
        print("landing successful")
        super().moat_exit_action()

    def goTo(self, dest: Pos, wp_type: int = 1) -> None:
        # FIXME These should be in the abstract base class?
        self.reached = False
        self.waypoint_count += 1

        print(self._sub_positioning.name, "going to point", dest)
        goal_pose = PoseStamped()
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.header.frame_id = "world"  # Frame ID must be "world"
        goal_pose.pose = dest.to_pose()

        # TODO wp_type is ignored for now. Should be handled

        self.__pose_client.wait_for_server()  # TODO Set Timeout?

        # This does not wait for result and will preempt previous goal.
        self.__pose_client.send_goal(
            hq_msg.PoseGoal(goal_pose),
            lambda status, result: self._getReached(str(wp_type == 1))
        )

    def follow_path(self, path: list) -> None:
        for wp in path[:-1]:
            self.goTo(wp, 0)
        self.goTo(path[-1], 1)

    def run(self):
        rospy.spin()
