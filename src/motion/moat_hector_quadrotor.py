"""
Motion Automaton implementation for Hector Quadrotor ROS package
http://wiki.ros.org/hector_quadrotor
"""

import numpy as np
from threading import RLock

from actionlib import SimpleActionClient, GoalStatus
from geometry_msgs.msg import PoseStamped
from hector_uav_msgs import msg as hq_msg
import rospy

from src.motion.motionautomaton import MotionAutomaton
from src.motion.pos_types import Pos


class MoatHectorQuadrotor(MotionAutomaton):

    def __init__(self, config):
        act_prefix = config.bot_name + "/action/"  # FIXME better way to find namespace for actions
        # XXX Takeoff and Pose has to be assigned before super().__init__
        #  because moat_init_action is called and uses it
        self.__takeoff_act = act_prefix + "takeoff"
        # __pose_client should only be used in MotionAutomaton thread to avoid concurrency bugs
        self.__pose_client = SimpleActionClient(act_prefix + "pose", hq_msg.PoseAction)

        super(MoatHectorQuadrotor, self).__init__(config)

        self.__landing_act = act_prefix + "landing"

        self.__path = None
        self.__path_lock = RLock()

        # TODO hector_quadrotor package already provides necessary actions,
        #  we may unregister unnecessary publishers and subscribers to save bandwidth

    # region called by ROS subscriber threads
    def _getPositioning(self, data) -> None:
        self.position = Pos(np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z]))
    # endregion

    # region called by AgentThread thread
    def moat_init_action(self):
        super().moat_init_action()

        goal_pose = PoseStamped()
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.header.frame_id = "world"  # Frame ID must be "world"
        goal_pose.pose = (self.position + Pos(np.array([0, 0, 0.5]))).to_pose()

        self.__pose_client.wait_for_server()  # TODO Set Timeout?
        goal = hq_msg.PoseGoal(goal_pose)
        print("taking off")
        result = self.__pose_client.send_goal_and_wait(goal)  # TODO Set Timeout?
        # TODO Check result
        if result == GoalStatus.SUCCEEDED:
            print("take off succeeded")
        else:
            print("take off failed")

    def moat_exit_action(self):
        client = SimpleActionClient(self.__landing_act, hq_msg.LandingAction)
        client.wait_for_server()  # TODO Set Timeout?
        goal = hq_msg.LandingGoal()

        print("landing")
        result = client.send_goal_and_wait(goal)  # TODO Set Timeout?
        # TODO check result
        if result == GoalStatus.SUCCEEDED:
            print("landing succeeded")
        else:
            print("landing failed")
        super().moat_exit_action()

    def goTo(self, dest: Pos, wp_type: int = 1) -> None:
        self.__put_way_points([(dest, wp_type)])

    def follow_path(self, path: list) -> None:
        wp_list = [(p, 0) for p in path[:-1]] + [(path[-1], 1)]
        self.__put_way_points(wp_list)

    def __put_way_points(self, wp_list: list) -> None:
        self.reached = False
        with self.__path_lock:
            if self.__path:
                print(self._sub_positioning.name, "Discard previous target")
            self.__path = wp_list
    # endregion

    # region called by MotionAutomaton thread
    def _getReached(self, data) -> None:
        if data.upper() == "TRUE":
            self.reached = True

    def __process_way_point(self, dest: Pos, wp_type: int):
        goal_pose = PoseStamped()
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.header.frame_id = "world"  # Frame ID must be "world"
        goal_pose.pose = dest.to_pose()

        if self.__pose_client.gh and \
                self.__pose_client.get_state() != GoalStatus.SUCCEEDED:
            print(self._sub_positioning.name, "preempting previous point")
        print(self._sub_positioning.name, "going to point", dest)
        self.__pose_client.wait_for_server()  # TODO Set Timeout?
        # This does not wait for result and will preempt previous goal.
        self.__pose_client.send_goal(
            hq_msg.PoseGoal(goal_pose),
            lambda status, result: self._getReached(str(wp_type == 1))
        )

    def run(self):
        try:
            prev_path = None
            while not rospy.is_shutdown():
                rospy.sleep(0)  # Yield to other threads
                with self.__path_lock:
                    if not self.__path:  # No target
                        continue
                    if self.__path is prev_path and \
                            self.__pose_client.get_state() != GoalStatus.SUCCEEDED:
                        # continue going to previous way point
                        continue
                    if self.__path is not prev_path:
                        iter_path = iter(self.__path)
                        prev_path = self.__path  # Record id of this __path
                    # else:  # process next way point
                    wp = next(iter_path, None)  # get next way point
                if not wp:
                    continue  # Finished all way points. Wait for next path
                # Process way points outside the lock so that
                # AgentThread may get the lock to override the __path
                dest, wp_type = wp
                self.__process_way_point(dest, wp_type)
        except KeyboardInterrupt:
            rospy.logdebug("keyboard interrupt, shutting down")
            rospy.signal_shutdown('keyboard interrupt')
    # endregion
