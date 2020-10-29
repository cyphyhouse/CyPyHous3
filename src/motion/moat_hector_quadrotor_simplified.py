from copy import deepcopy
from queue import Queue

import numpy as np

from src.motion.motionautomaton import MotionAutomaton
from src.motion.pos_types import Pos

RESEND_TIMEOUT = 5  # sec


class MoatHectorQuadrotorSimplified(MotionAutomaton):

    def __init__(self, config):
        # Thread safe Queue is required because pub and sub can be in
        # different threads
        self._waypoints = Queue()  # type: Queue

        self._init_z = 0.2

        super(MoatHectorQuadrotorSimplified, self).__init__(config)

    def _getPositioning(self, data) -> None:
        self.position = Pos(np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z]))

    def _getReached(self, data) -> None:
        a = str(data).upper()
        if 'TRUE' in a:
            self.reached = True

    def takeoff(self):
        print("taking off")
        takeoff = Pos(np.array([self.position.x, self.position.y, 1.0]))
        self.goTo(takeoff)

    def land(self):
        print("landing")
        landing = Pos(np.array([self.position.x, self.position.y, self._init_z]))
        self.goTo(landing)

    def moat_init_action(self):
        super(MoatHectorQuadrotorSimplified, self).moat_init_action()
        self._init_z = self.position.z
        import rospy
        self.takeoff()
        while not self.reached:
            rospy.sleep(1.0)
        print("take off successful")
        pass

    def moat_exit_action(self):
        import rospy
        # TODO: maybe incorporate call to best here?
        self.land()
        while not self.reached:
            rospy.sleep(1.0)
        print("landing successful")
        super(MoatHectorQuadrotorSimplified, self).moat_exit_action()

    def goTo(self, dest: Pos, wp_type: int = None) -> None:
        print("going to point", dest)
        if wp_type is not None:
            frame_id = str(wp_type)
        else:
            frame_id = '1'

        import rospy
        from geometry_msgs.msg import PoseStamped

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = frame_id
        pose.pose = dest.to_pose()

        self.reached = False

        self.waypoint_count += 1
        self._waypoints.put(pose)  # Mimic publishing to waypoint

    def follow_path(self, path: list) -> None:
        for wp in path[:-1]:
            self.goTo(wp, 0)
        self.goTo(path[-1], 1)

    def run(self):
        from geometry_msgs.msg import PoseStamped
        from hector_uav_msgs.srv import EnableMotors
        import rospy

        # Enable motors using ROS service
        service_name = rospy.resolve_name("enable_motors", self.rospy_node)
        rospy.loginfo("Wait for service server")
        rospy.wait_for_service(service_name, 20)
        try:
            # Set 'enable_motors' to be True
            enable_motor = rospy.ServiceProxy(service_name, EnableMotors)
            resp = enable_motor(True)
            if not resp.success:
                pass  # TODO What to do if motor is not enabled
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed with %s", e)

        cmd_pose_topic = rospy.resolve_name("command/pose", self.rospy_node)
        pub_cmd_pose = rospy.Publisher(cmd_pose_topic, PoseStamped, queue_size=1)

        def publish_pose(target_pose):
            tmp_pose = deepcopy(target_pose)
            tmp_pose.header.frame_id = "world"
            rospy.logdebug("Sending target pose\n %s" % str(tmp_pose))
            pub_cmd_pose.publish(tmp_pose)

        rate = rospy.Rate(100)  # 100Hz TODO Pass sleep rate as a parameter?
        is_driving = False
        curr_waypoint = None  # type: PoseStamped
        resend_after = rospy.get_rostime()
        start = rospy.get_rostime()
        while not rospy.is_shutdown():
            rate.sleep()
            # Simple controller code for drones # TODO Need better controller
            if not is_driving:  # IDLE
                if self._waypoints.empty():
                    pass  # Keep idling
                else:
                    curr_waypoint = self._waypoints.get()
                    publish_pose(curr_waypoint)
                    resend_after = rospy.get_rostime() + rospy.Duration(secs=RESEND_TIMEOUT)
                    is_driving = True
            else:  # DRIVING
                assert curr_waypoint is not None
                pos_target = Pos(np.array([curr_waypoint.pose.position.x,
                                           curr_waypoint.pose.position.y,
                                           curr_waypoint.pose.position.z,
                                           curr_waypoint.pose.orientation.z]))
                if not pos_within_tolerance(self.position, pos_target):
                    # Keep driving
                    if resend_after and rospy.get_rostime() > resend_after:
                        publish_pose(curr_waypoint)
                        resend_after = rospy.get_rostime() + rospy.Duration(secs=RESEND_TIMEOUT)
                else:
                    rospy.logdebug("Reached target pose\n %s" % str(curr_waypoint.pose))
                    if curr_waypoint.header.frame_id == "1":
                        self._getReached("TRUE")  # Mimic publishing the reached message
                    curr_waypoint = None
                    is_driving = False


def pos_within_tolerance(pos_current: Pos, pos_target: Pos,
                         dist_tolerance: float = 0.20,
                         yaw_tolerance: float = 0.35) -> bool:
    if dist_tolerance <= 0:
        raise ValueError("Distance tolerance must be greater than 0.")
    pos_diff_arr = (pos_target - pos_current).mk_arr()
    return np.linalg.norm(pos_diff_arr[0:3]) <= dist_tolerance
