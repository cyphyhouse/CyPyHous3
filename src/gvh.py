# from typing import NoReturn
import typing
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
        try:
            import motionAutomaton
            self.__moat = motionAutomaton.MotionAutomaton(pid, bot_name)
            self.__moat.start()
        except ImportError:
            print("maybe you dont have ros installed")
            self.__moat = None

    @property
    def moat(self) -> typing.Any:
        """
        getter method for motionAutomaton
        :return:
        """
        return self.__moat

    @moat.setter
    def moat(self, moat: typing.Any) -> None:  # -> NoReturn:
        """
        setter method for moAT
        :param moat: motionautomaton
        :return:
        """
        self.__moat = moat

    @property
    def participants(self) -> int:  # -> NoReturn:
        """
        getter method for participants
        :return:
        """
        return self.__participants

    @participants.setter
    def participants(self, participants: int) -> None:  # -> NoReturn:
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
    def pid(self, pid: int) -> None:  # -> NoReturn:
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
    def agent_dsm(self, my_dsm: Dsm) -> None:
        """
        setter method for dsm
        :param mydsm:
        :return:
        """
        self.__dsm = my_dsm

    def mk_var(self, var_scope, var_type, var_name, var_value=None) -> None:
        """
        method to create variable type in dsm
        :param var_scope: scope of variable
        :param var_type: variable type
        :param var_name: name
        :param var_value: value
        :return: nothing
        """
        if var_scope == 'ar':
            self.agent_dsm.mk_ar_var(self.pid, self.participants, var_type, var_name, var_value)
        elif var_scope == 'aw':
            self.agent_dsm.mk_aw_var(var_type, var_name, var_value)
