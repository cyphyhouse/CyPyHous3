# Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

import typing as tp

from src.objects.abstract.dsm import dsm


class dsmAllWrite(dsm):
    """
    __value : value
    __last_updated : last updated time stamp

    """

    def __init__(self, name: str, data_type: type,
                 value: tp.Union[int, bool, float, list, object, tuple, None] = None,
                 last_updated: float = 0.0):
        super().__init__(name, data_type)
        self.__value = value
        self.__last_updated = last_updated

    def __repr__(self):
        """
        string representation
        """
        return str(self.name) + " " + str(self.get_val())

    # ------------ MEMBER ACCESS METHODS--------------

    def last_update(self) -> float:
        """
        getter method for timestamp of last update

        :param pid: pid of agent to get the update time for
        :type pid: int

        :return: update time stamp
        :rtype: float
        """
        return self.__last_updated

    def set_update(self, update_time_stamp: float) -> None:
        """
        setter method for timestamp of last update

        :param update_time_stamp: updated time stamp
        :type update_time_stamp: float

        """
        self.__last_updated = update_time_stamp

    def get_val(self) -> tp.Union[
        int, bool, float, list, object, tuple, None]:  # -> Any (not available in python 3.5)
        """
        getter method for value of shared

        :return: value of shared variable
        :rtype: int,bool,float,list,object,tuple,None
        """
        return self.__value

    def set_val(self, value: tp.Union[int, bool, float, list, object, tuple]) -> None:
        """
        setter method for value of shared variable
        :param value : value to be set
        :type value: tp.Union[int, bool, float, list, object, tuple])
        """
        self.__value = value
