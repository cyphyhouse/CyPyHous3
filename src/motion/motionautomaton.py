import threading
import time
from abc import ABC, abstractmethod
from enum import Enum
from typing import Union

from src.motion.planner import Planner
from src.motion.pos import Pos
from src.motion.simpleplanner import SimplePlanner


class BotType(Enum):
    """
    add more bot types here
    """
    QUAD = 1
    CAR = 2


class MoatConfig(object):

    def __init__(self, waypoint_topic: str, reached_topic: str, rospy_node: str, positioning_params: tuple,
                 reached_params: tuple, bot_name: str, queue_size: int, bot_type: BotType,
                 pub_msg_type=None, planner: Planner = SimplePlanner()):
        """

        :param waypoint_topic:
        :param reached_topic:
        :param rospy_node:
        :param positioning_params:
        :param bot_name:
        :param queue_size:
        :param bot_type:
        :param reached_params:
        :param msg_type:
        :param planner:
        """
        self.waypoint_topic = waypoint_topic
        self.reached_topic = reached_topic
        self.reached_params = reached_params
        self.positioning_params = positioning_params
        self.msg_type = msg_type
        self.rospy_node = rospy_node
        self.bot_name = bot_name
        self.queue_size = queue_size
        self.bot_type = bot_type
        self.planner = planner


def gen_positioning_params(node_name, bot_name, msg_type):
    return node_name + bot_name + '/pose', msg_type


def gen_reached_params(reached_topic, msg_type):
    return reached_topic, msg_type

def default_car_moat_config(bot_name) -> MoatConfig:
    rospy_node = 'quad_wp_node'
    bot_type = BotType.CAR
    waypoint_topic = 'Waypoint'
    reached_topic = '/Reached'

    from geometry_msgs import PoseStamped
    from std_msgs.msg import String

    pub_msg_type = PoseStamped
    queue_size = 1
    bot_name = bot_name
    positioning_params = gen_positioning_params('/vrpn_client_node/', bot_name, PoseStamped)
    reached_params = gen_reached_params(reached_topic, String)
    return MoatConfig(waypoint_topic, reached_topic, rospy_node, positioning_params, reached_params, bot_name,
                      queue_size, bot_type, pub_msg_type, SimplePlanner())

def default_qc_moat_config(bot_name, positioning_callback, reached_callback) -> MoatConfig:
    rospy_node = 'quad_wp_node'
    bot_type = BotType.CAR
    waypoint_topic = 'Waypoint'
    reached_topic = '/Reached'

    from geometry_msgs import PoseStamped
    from std_msgs.msg import String

    pub_msg_type = PoseStamped
    queue_size = 1
    bot_name = bot_name
    positioning_params = gen_positioning_params('/vrpn_client_node/', bot_name, PoseStamped)
    reached_params = gen_reached_params(reached_topic, String)
    return MoatConfig(waypoint_topic, reached_topic, rospy_node, positioning_params, reached_params, bot_name,
                      queue_size, bot_type, pub_msg_type, SimplePlanner())


class MotionAutomaton(threading.Thread, ABC):

    def __init__(self, config):
        # TODO: work on all the configs, print initialization messages
        # print("initializing motion automaton")
        threading.Thread.__init__(self)
        self.__waypoint_count = 0
        self.__position = Pos()
        self.__reached = False
        self.__path = []
        self.__planner = config.planner
        self.__bot_type = config.bot_type

        try:
            import rospy
            rospy.init_node(config.rospy_node, anonymous=True)
            self.__pub = rospy.Publisher(config.waypoint_topic, config.pub_msg_type, queue_size=config.queue_size)
            self.__sub_reached = rospy.Subscriber(*config.reached_params, self._getReached() , queue_size=config.queue_size)
            self.__sub_positioning = rospy.Subscriber(*config.positioning_params, self._getPositioning(), queue_size=config.queue_size)

        except ImportError:
            self.__pub = None
            self.__sub_reached = None
            self.__sub_positioning = None
            print("maybe issue with ros installation")

        time.sleep(1)

    @property
    def path(self) -> Union[Pos, list]:
        """
        getter method for path
        :return:
        """
        if self.__path is not []:
            return self.__path
        else:
            return self.position

    @path.setter
    def path(self, path):
        """
        setter method for path
        :param path:
        :return:
        """
        self.__path = path

    @property
    def pub(self):
        """
        getter method for publisher
        :return:
        """
        return self.__pub

    @property
    def position(self) -> Pos:
        """
        getter method for position"
        :return:
        """
        return self.__position

    @position.setter
    def position(self, pos: Pos) -> None:
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
    def waypoint_count(self, wpc: int) -> None:
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
    def reached(self, r: bool) -> None:
        """
        setter method for reached
        :param r: bool
        :return:
        """
        self.__reached = r

    @property
    def bot_type(self):
        """
        getter method for bot type
        :return:
        """
        return self.__bot_type

    @bot_type.setter
    def bot_type(self, bot_type):
        """
        setter method for bot type
        :param bot_type:
        :return:
        """
        self.__bot_type = bot_type

    @property
    def planner(self):
        """
        getter method for planner
        :return:
        """
        return self.__planner

    @planner.setter
    def planner(self, p):
        """
        setter method for planner
        :param p:
        :return:
        """
        self.__planner = p

    @abstractmethod
    def _getPositioning(self, data) -> Pos:
        """
        This is a callback function that updates the internal position and heading,
        :param data: position message.
        :return:
        """
        pass

    @abstractmethod
    def _getReached(self, data) -> bool:
        """
        callback function that updates the reached flag
        :param data:
        :return:
        """
        pass

    @abstractmethod
    def goTo(self, dest: Pos, wp_type: int = None) -> None:
        """
        goto position. should publish correct ros message
        :param dest:
        :param wp_type:
        :return:
        """
        pass

    @abstractmethod
    def follow_path(self, path: list) -> None:
        """
        Follow defined path
        :param path:
        :return:
        """
        pass

    @abstractmethod
    def run(self):

        """
        abstract method for running the motion automaton thread
        calls the spin function to check for new vicon data
        :return:
        """
        pass
