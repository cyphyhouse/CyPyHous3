import threading
import time

import rospy
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String


class MotionAutomaton(threading.Thread):
    """
    __waypoint_count: int
    __position: Pose
    __reached: bool
    __pub: rospy.Publisher
    __sub_vicon: rospy.Subscriber

    """

    def __init__(self, planner, bot_num: int = 1, bot_name: str = 'cyphyhousecopter', queue_size=1):
        print("calling init")
        threading.Thread.__init__(self)
        self.__waypoint_count = 0
        self.__position = Pose()
        self.__reached = False
        self.__path = []
        self.__planner = planner

        rospy.init_node('quad_wp_node', anonymous=True)
        self.__pub = rospy.Publisher('Waypoint_bot' + str(bot_num), PoseStamped, queue_size=queue_size)
        self.__sub_vicon = rospy.Subscriber('/vrpn_client_node/' + bot_name + '/pose', PoseStamped, self._getVicon,
                                            queue_size=1)
        self.__sub_reached = rospy.Subscriber('/Reached', String, self._getReached, queue_size=1)
        time.sleep(1)

    @property
    def planner(self):
        return self.__planner

    @planner.setter
    def planner(self, p):
        self.__planner = p

    @property
    def path(self):
        if self.__path is not []:
            return self.__path
        else:
            return self.position

    @path.setter
    def path(self, path):
        self.__path = path

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

    @reached.setter
    def reached(self, r: bool):  # -> NoReturn:
        """
        setter method for reached
        :param r: bool
        :return:
        """
        self.__reached = r

    def _getVicon(self, data: PoseStamped):  # -> NoReturn:
        """
        This is a callback function that updates the internal position and heading,
        :param data: posestamped message.
        :return:
        """
        self.position = data.pose

    def _getReached(self, data:  str):  # -> NoReturn:
        """
        This is a callback function that updates the internal Reached state
        :param data: String message
        :return: None
        """
        print(type(data))
        a = str(data).upper()
        print(a)
        if a == 'TRUE':
            self.reached = True

    def goTo(self, dest: Pose, wp_type: int = None):  # -> NoReturn:
        """
        takes a geometry message and uses on board controller to move to it.
        :param dest: destination position
        :param wp_type: to pick frameid for ros geometry message.
        :return:
        """
        print("going to point", dest)
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

    def follow_path(self, path) -> None:
        """
        Follow paths in CyPhyHouse by sending goTo message to vehicle
        """
        # Initialize list of waypoints
        wp_list = []
        for point in path:
            point = point.topoint()
            wp = Pose()
            wp.position.x = point[0]
            wp.position.y = point[1]
            wp.position.z = point[2]
            wp_list.append(wp)
        for wp in wp_list[:-1]:
            self.goTo(wp, 0)
        self.goTo(wp_list[-1], 1)



    def run(self):
        """
        calls the spin function to check for new vicon data
        :return:
        """
        rospy.spin()
