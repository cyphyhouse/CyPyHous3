# Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

import typing as tp

from src.objects.abstract.dsm import dsm


class dsmAllRead(dsm):

    def __init__(self, name: str, data_type: type, size: int, pid: int,
                 value: tp.Union[int, bool, float, list, object, tuple, None] = None,
                 last_updated: float = 0.0):
        """
        :param name: variable name
        :type name: str

        :param data_type: datatype of variable
        :type data_type: type

        :param size: number of robots in system
        :type size: int

        :param pid: pid of current robot
        :type pid: int

        :param value: value of variable
        :type value: [int,bool,float,list,object,tuple,None]

        :param last_updated: timestamp of last update
        :type last_updated: float
        """

        super().__init__(name, data_type)
        self.__pid = pid
        self.__value_dict = {}
        self.__last_updated_dict = {}
        for i in range(size):
            self.__value_dict[i] = value
            self.__last_updated_dict[i] = last_updated

    def __repr__(self):
        """
        string representation
        """
        return str(self.name) + " " + str(self.get_val(self.pid)) + " for " + str(self.pid)

    # ------------ MEMBER ACCESS METHODS --------------

    @property
    def pid(self) -> int:
        return self.__pid

    def last_update(self, pid: int) -> float:
        """
        getter method for timestamp of last update

        :param pid: pid of agent to get the update time for
        :type pid: int

        :return: update time stamp
        :rtype: float,dict
        """
        return self.__last_updated_dict[pid]

    def set_update(self, update_time_stamp: float, pid: int) -> None:
        """
        setter method for timestamp of last update

        :param update_time_stamp: updated time stamp
        :type update_time_stamp: float

        :param pid: pid of agent to set the update time for
        :type pid: int
        """
        self.__last_updated_dict[pid] = update_time_stamp

    def get_val(self, pid: int) -> tp.Union[
        int, bool, float, list, object, tuple, None]:  # -> Any (not available in python 3.5)
        """
        getter method for value of shared

        :param pid: pid of agent to get the value of
        :type pid: int

        :return: value of shared variable for pid
        :rtype: int,bool,float,list,object,tuple,None
        """
        return self.__value_dict[pid]

    def set_val(self, value: tp.Union[int, bool, float, list, object, tuple], pid: int) -> None:
        """
        setter method for value of shared variable
        :param value : value to be set
        :param pid: pid of agent to set the value of
        """
        self.__value_dict[pid] = value
