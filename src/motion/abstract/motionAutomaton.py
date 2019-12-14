#  Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

import abc
import threading
import time
import typing

import src.config.configs as configs
import src.datatypes.motion.pos as pos


class MotionAutomaton(threading.Thread, abc.ABC):

    def __init__(self, config: configs.MoatConfig):
        """
        :param config: motion automaton configuration
        :type config: MoatConfig
        """
        threading.Thread.__init__(self)
        self.__way_point_count = 0

        self.__reached = False
        self.__position = None
        self.__path = []
        self.__planner = config.planner
        self.__bot_type = config.bot_type

        try:
            import rospy
            rospy.init_node(config.ros_py_node, anonymous=True)
            self.__pub = rospy.Publisher(*configs.gen_way_point_params(config), queue_size=config.queue_size)
            self.__sub_reached = rospy.Subscriber(*configs.gen_reached_params(config), self._get_reached,
                                                  queue_size=config.queue_size)
            self.__sub_positioning = rospy.Subscriber(*configs.gen_positioning_params(config), self._get_positioning,
                                                      queue_size=config.queue_size)

        except ImportError:
            rospy = None
            self.__pub = None
            self.__sub_reached = None
            self.__sub_positioning = None
            print("maybe issue with ros installation")

        # need *a* sleep to only start motion automaton after subscriber starts listening
        time.sleep(1)
        self.moat_init_action()

    # ------------ MOTION AUTOMATON STARTUP AND EXIT METHODS --------------

    @abc.abstractmethod
    def moat_init_action(self):
        pass

    @abc.abstractmethod
    def moat_exit_action(self):
        pass

    # ------------ MEMBER ACCESS METHODS --------------

    @property
    def path(self) -> typing.Union[pos.Pos, typing.List[pos.Pos]]:
        if self.__path is not []:
            return self.__path
        else:
            return [self.position]

    @path.setter
    def path(self, path):
        self.__path = path

    @property
    def pub(self):
        return self.__pub

    @property
    def position(self) -> pos.Pos:
        return self.__position

    @position.setter
    def position(self, new_pos: pos.Pos) -> None:
        self.__position = new_pos

    @property
    def way_point_count(self) -> int:
        return self.__way_point_count

    @way_point_count.setter
    def way_point_count(self, wpc: int) -> None:
        self.__way_point_count = wpc

    @property
    def reached(self) -> bool:
        return self.__reached

    @reached.setter
    def reached(self, r: bool) -> None:
        self.__reached = r

    @property
    def bot_type(self):
        return self.__bot_type

    @bot_type.setter
    def bot_type(self, bot_type):
        self.__bot_type = bot_type

    @property
    def planner(self):
        return self.__planner

    @planner.setter
    def planner(self, p):
        self.__planner = p

    # ------------ CALLBACK METHODS FOR POSITIONING AND REACHED METHODS --------------

    @abc.abstractmethod
    def _get_positioning(self, data) -> pos.Pos:
        pass

    @abc.abstractmethod
    def _get_reached(self, data) -> bool:
        pass

    # ------------ ABSTRACT METHOD FOR  GOING TO POINT AND FOLLOWING PATH --------------

    @abc.abstractmethod
    def go_to(self, dest: pos.Pos, wp_type: int = None) -> None:
        pass

    @abc.abstractmethod
    def follow_path(self, path: typing.List[pos.Pos]) -> None:
        pass

    # ------------ THREAD BEHAVIOR --------------

    @abc.abstractmethod
    def run(self):
        """
        abstract method for running the motion automaton thread
        """
        pass
