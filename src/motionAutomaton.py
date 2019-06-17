import threading
import time
import rospy
from geometry_msgs.msg import PoseStamped, Pose


class MotionAutomaton(threading.Thread):
    """
    __waypoint_count: int
    __position: Pose
    __reached: bool
    __pub: rospy.Publisher
    __sub_vicon: rospy.Subscriber

    """

    def __init__(self, bot_num: int = 1, bot_name: str = 'cyphyhousecopter'):
        threading.Thread.__init__(self)
        self.__waypoint_count = 0
        self.__position = Pose()
        self.__reached = False

        rospy.init_node('quad_wp_node', anonymous=True)
        self.__pub = rospy.Publisher('Waypoint_bot' + str(bot_num), PoseStamped, queue_size=1)
        self.__sub_vicon = rospy.Subscriber('/vrpn_client_node/' + bot_name + '/pose', PoseStamped, self._getVicon,
                                            queue_size=1)
        self.__sub_reached = rospy.Subscriber('/Reached', String, self._reachedCB, queue_size=1)
        time.sleep(1)

    @property
    def pub(self):
        """
        getter method for publisher
        :return:
        """
        return self.__pub

    @property
    def position(self) -> Pose:
        """
        getter method for position"
        :return:
        """
        return self.__position

    @position.setter
    def position(self, pos: Pose):  # -> NoReturn:
        """
        setter method for position
        :param pos: position
        :return:
        """
        self.__position = pos

    @property
    def waypoint_count(self) -> int:
        """
        current method of figuring out whether the current point is a takeoff point
        :return:
        """
        return self.__waypoint_count

    @waypoint_count.setter
    def waypoint_count(self, wpc: int):  # -> NoReturn:
        """
        setter method for an internal function
        :return:
        """
        self.__waypoint_count = wpc

    @property
    def reached(self) -> bool:
        """
        getter method for reached"
        :return:
        """
        return self.__reached

    @position.setter
    def reached(self, r: bool):  # -> NoReturn:
        """
        setter method for reached
        :param r: bool
        :return:
        """
        self.__reached = r

    def _reachedCB(self, data: String):  # -> NoReturn:
        """
        This is a callback function that updates the internal Reached state
        :param data: String message
        :return: None
        """
        if data.lower() == 'true':
            self.reached = True

    def _getVicon(self, data: PoseStamped):  # -> NoReturn:
        """
        This is a callback function that updates the internal position and heading,
        :param data: posestamped message.
        :return:
        """
        self.position = data.pose

    def goTo(self, dest: Pose, wp_type: int = None):  # -> NoReturn:
        """
        takes a geometry message and uses on board controller to move to it.
        :param dest: destination position
        :param wp_type: to pick frameid for ros geometry message.
        :return:
        """
        if wp_type is not None:
            frame_id = str(wp_type)
        else:
            if self.waypoint_count == 0:
                frame_id = '0'
            elif dest.position.z <= 0:
                frame_id = '2'
            else:
                frame_id = '1'

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = frame_id
        pose.pose = dest

        self.reached = False

        self.waypoint_count += 1
        self.pub.publish(pose)

    def stop(self):
        rospy.signal_shutdown("rospy shutdown: end of program reached")

    def run(self):
        """
        calls the spin function to check for new vicon data
        :return:
        """
        rospy.spin()
        print("ros stopped")
