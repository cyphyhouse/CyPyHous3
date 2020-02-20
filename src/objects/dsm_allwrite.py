# Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

import typing as tp


class dsmAllWrite(object):
    """
    distributed shared memory object
    __name : variable name
    __data_type : data type
    __value : value
    __last_updated : last updated time stamp


    """

    def __init__(self, name: str, data_type: type,
                 value: tp.Union[int, bool, float, list, object, tuple, None] = None,
                 last_updated: float = 0.0):
        """

        :param name: variable name
        :type name: str

        :param data_type: variable datatype
        :type data_type: type

        :param size: number of agents
        :type size: int

        :param pid: unique integer identifier of current agent
        :type pid: int

        :param value: value of dsm
        :type value: int, bool, float, list, object, tuple, None

        :param share_scope: shared variable scope
        :type share_scope: ShareType

        :param last_updated: last update
        :type last_updated: float

        """
        self.name = name
        self.data_type = data_type
        self.__value = value
        self.__last_updated = last_updated

    def __repr__(self):
        """
        string representation
        """
        return str(self.name) + " " + str(self.get_val())

        # ------------ MEMBER ACCESS METHODS --------------

    @property
    def name(self) -> str:
        return self.__name

    @name.setter
    def name(self, name: str) -> None:
        self.__name = name

    @property
    def data_type(self) -> type:
        return self.__data_type

    @data_type.setter
    def data_type(self, data_type: type) -> None:
        self.__data_type = data_type

    # ------------ MEMBER ACCESS METHODS FOR POTENTIALLY LIST ITEMS--------------

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
