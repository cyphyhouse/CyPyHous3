# from typing import NoReturn
import typing

import mutex
from dsm import Dsm
import message


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
        self.__mutex_list = []
        self.__pid = pid
        self.__dsm = Dsm()
        self.__msg_list = []

        if pid == 0:
            self.__is_leader = True
        else:
            self.__is_leader = False
        try:
            import motionAutomaton
            self.__moat = motionAutomaton.MotionAutomaton(pid, bot_name)
            self.__moat.start()
        except ImportError:
            print("maybe you dont have ros installed")
            self.__moat = None

    @property
    def msg_list(self) -> list:
        """

        :return:
        """
        return self.__msg_list

    @msg_list.setter
    def msg_list(self, msg_list: list) -> None:
        """

        :param msg_list:
        :return:
        """
        self.__msg_list = msg_list

    @property
    def mutex_list(self) -> list:
        """

        :return:
        """
        return self.__mutex_list

    @mutex_list.setter
    def mutex_list(self, mutex_list: list) -> None:
        """

        :type mutexlist: list
        """
        self.__mutex_list = mutex_list

    @property
    def is_leader(self) -> bool:
        """

        :return:
        """
        return self.__is_leader

    @is_leader.setter
    def is_leader(self, val: bool) -> None:
        """

        :param val:
        :return:
        """
        self.__is_leader = val

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
            self.mutex_list.append(mutex.Mutex(var_name))
        elif var_scope == 'aw':
            self.agent_dsm.mk_aw_var(var_type, var_name, var_value)
            self.mutex_list.append(mutex.Mutex(var_name))

    def get_mutex_index(self, var_name: str) -> typing.Union[int, None]:
        """

        :param var_name:
        :return:
        """
        for i in range(len(self.mutex_list)):
            if self.mutex_list[i].var_name == var_name:
                return i
        return None

    def has_mutex(self, var_name):
        """

        :param var_name:
        :return:
        """
        i = self.get_mutex_index(var_name)
        if self.mutex_list[i].has_mutex(self.pid):
            return True
        return False

    def add_msg(self, msg: message.Message) -> None:
        """
        add message to list
        :param msg:
        :return:
        """
        self.msg_list.append(msg)
