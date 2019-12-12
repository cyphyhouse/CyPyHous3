#  Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

import threading
import time
from abc import ABC, abstractmethod
from typing import Union, List

from src.config.configs import MoatConfig, gen_positioning_params, gen_reached_params, gen_way_point_params
from src.datatypes.motion.pos import Pos


class MotionAutomaton(threading.Thread, ABC):

    def __init__(self, config: MoatConfig):
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
            self.__pub = rospy.Publisher(*gen_way_point_params(config), queue_size=config.queue_size)
            self.__sub_reached = rospy.Subscriber(*gen_reached_params(config), self._get_reached,
                                                  queue_size=config.queue_size)
            self.__sub_positioning = rospy.Subscriber(*gen_positioning_params(config), self._get_positioning,
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

    @abstractmethod
    def moat_init_action(self):
        pass

    @abstractmethod
    def moat_exit_action(self):
        pass

    # ------------ MEMBER ACCESS METHODS --------------

    @property
    def path(self) -> Union[Pos, List[Pos]]:
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
    def position(self) -> Pos:
        return self.__position

    @position.setter
    def position(self, pos: Pos) -> None:
        self.__position = pos

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

    @abstractmethod
    def _get_positioning(self, data) -> Pos:
        pass

    @abstractmethod
    def _get_reached(self, data) -> bool:
        pass

    # ------------ ABSTRACT METHOD FOR  GOING TO POINT AND FOLLOWING PATH --------------

    @abstractmethod
    def go_to(self, dest: Pos, wp_type: int = None) -> None:
        pass

    @abstractmethod
    def follow_path(self, path: List[Pos]) -> None:
        pass

    # ------------ THREAD BEHAVIOR --------------

    @abstractmethod
    def run(self):

        """
        abstract method for running the motion automaton thread
        """
        pass
