import threading
import time
from abc import ABC, abstractmethod
from typing import Union

from src.config.configs import MoatConfig, gen_positioning_params,gen_reached_params, gen_waypoint_params
from src.motion.pos_types import Pos


class MotionAutomaton(threading.Thread, ABC):

    def __init__(self, config: MoatConfig):
        # TODO: work on all the configs, print initialization messages
        # print("initializing motion automaton")
        threading.Thread.__init__(self)
        self.__waypoint_count = 0

        self.__reached = False
        self.__position = None
        self.__path = []
        self.__planner = config.planner
        self.__bot_type = config.bot_type

        try:
            import rospy
            rospy.init_node("motionautomaton", anonymous=True, disable_signals=True)
            self.__pub = rospy.Publisher(*gen_waypoint_params(config), queue_size=config.queue_size)
            self.__sub_reached = rospy.Subscriber(*gen_reached_params(config), self._getReached,
                                                  queue_size=config.queue_size)
            self._sub_positioning = rospy.Subscriber(*gen_positioning_params(config), self._getPositioning,
                                                     queue_size=config.queue_size)

        except ImportError:
            self.__pub = None
            self.__sub_reached = None
            self._sub_positioning = None
            print("maybe issue with ros installation")

        time.sleep(1)
        self.moat_init_action()

    def reset(self) -> None:
        pass

    @abstractmethod
    def moat_init_action(self):
        """
        action to perform when the motion automaton starts up.
        :return:
        """
        import rospy
        if self.__position:
            return
        # else wait for messages to initialize position
        rospy.loginfo("Waiting for the initial position")
        timeout=10
        try:
            rospy.wait_for_message(
                self._sub_positioning.name,
                self._sub_positioning.data_class,
                timeout=timeout
            )
            # self.__sub_positioning should also receive the message now.
            # Just yield so the subscriber thread can update the position
            rospy.sleep(0)
        except rospy.exceptions.ROSException:
            rospy.logerr("Unable to initialize position after %d sec. Shutdown ROS node for motion automaton.", timeout)
            rospy.signal_shutdown("Unable to intialize position.")
            raise NotImplementedError("TODO handle initialization failure")


    @abstractmethod
    def moat_exit_action(self):
        """
        action to perform when the motion automaton exits
        :return:
        """
        import rospy
        rospy.signal_shutdown("Motion automaton exit action. Shutting down...")

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
