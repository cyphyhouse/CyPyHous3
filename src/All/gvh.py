# from typing import NoReturn

import motionAutomaton
from dsm import Dsm


class Gvh(object):
    """
    __participants: int
    __pid: int
    __dsm: Dsm
    __moat: motionAutomaton.MotionAutomaton
    """

    def __init__(self, pid: int, participants: int = 1, bot_name: str = 'cyphyhousecopter'):
        """
        Global Variable Holder keeps system specific information in the robot
        :param name: the robots pid
        :param participants: all the other participants
        """
        self.__participants = participants
        self.__pid = pid
        self.__dsm = Dsm()
        self.__moat = motionAutomaton.MotionAutomaton(pid, bot_name)
        self.__moat.start()

    @property
    def moat(self) -> motionAutomaton.MotionAutomaton:
        """
        getter method for motionAutomaton
        :return:
        """
        return self.__moat

    @moat.setter
    def moat(self, moat: motionAutomaton.MotionAutomaton):  # -> NoReturn:
        """
        setter method for moAT
        :param moat: motionautomaton
        :return:
        """
        self.__moat = moat

    @property
    def participants(self):  # -> NoReturn:
        """
        getter method for participants
        :return:
        """
        return self.__participants

    @participants.setter
    def participants(self, participants: int):  # -> NoReturn:
        """
        setter method for participants
        :param name:
        :return:
        """
        self.__participants = participants

    @property
    def pid(self) -> int:
        """
        getter method for pid
        :return:
        """
        return self.__pid

    @pid.setter
    def pid(self, pid: int):  # -> NoReturn:
        """
        setter method for pid
        :param name:
        :return:
        """
        self.__pid = pid

    @property
    def agent_dsm(self) -> Dsm:
        """
        getter method for dsm
        :return:
        """
        return self.__dsm

    @agent_dsm.setter
    def agent_dsm(self, my_dsm: Dsm):
        """
        setter method for dsm
        :param mydsm:
        :return:
        """
        self.__dsm = my_dsm
