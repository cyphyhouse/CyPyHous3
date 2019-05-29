from typing import NoReturn
from dsm import Dsm
from motionautomaton import MotionAutomaton

class Gvh(object):
    __participants: int
    __pid: int
    __dsm: Dsm
    __moat:

    def __init__(self, pid: int, participants: int = 1, vtype: int):
        """
        Global Variable Holder keeps system specific information in the robot
        :param name: the robots pid
        :param participants: all the other participants
        """
        self.__participants = participants
        self.__pid = pid
        self.__dsm = Dsm()


    @property
    def participants(self) -> NoReturn:
        """
        getter method for participants
        :return:
        """
        return self.__participants

    @participants.setter
    def participants(self, participants: int) -> NoReturn:
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
    def pid(self, pid: int) -> NoReturn:
        """
        setter method for pid
        :param name:
        :return:
        """
        self.__pid = pid
