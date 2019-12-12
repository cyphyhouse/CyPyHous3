# Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

import typing as tp

import src.datatypes.var_types as vt


class DSM(object):
    """
    distributed shared memory object
    __name : variable name
    __data_type : data type
    __size : size (number of robots in the system/sharing the variable, if ALL_READ)
    __pid : pid of agent with corresponding dsm
    __value : value
    __last_updated : last updated time stamp
    __owner : variable sharing scope.


    """

    def __init__(self, name: str, data_type: type, size: int, pid: int,
                 value: tp.Union[int, bool, float, list, object, tuple, None] = None,
                 share_scope: vt.ShareType = vt.ShareType.ALL_WRITE,
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
        self.__name = name
        self.__data_type = data_type
        self.__size = size
        self.__pid = pid
        if share_scope == vt.ShareType.ALL_WRITE:
            self.__value = value
            self.__last_updated = last_updated
        else:
            self.__value = {}
            self.__last_updated = {}
            for i in range(size):
                if i == pid:
                    self.__value[i] = value
                else:
                    self.__value[i] = None
                self.__last_updated[i] = last_updated
        self.__owner = share_scope

    def __repr__(self):
        """
        string representation
        """
        return str(self.name) + " " + str(self.get_val(self.pid)) + " for " + str(self.pid)

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

    @property
    def owner(self) -> vt.ShareType:
        return self.__owner

    @owner.setter
    def owner(self, owner: vt.ShareType) -> None:
        self.__owner = owner

    @property
    def pid(self) -> int:
        return self.__pid

    @pid.setter
    def pid(self, pid: int) -> None:
        self.__pid = pid

    # ------------ MEMBER ACCESS METHODS FOR POTENTIALLY LIST ITEMS--------------

    def last_update(self, pid: int = -1) -> tp.Union[float, tp.Dict]:
        """
        getter method for timestamp of last update

        :param pid: pid of agent to get the update time for
        :type pid: int

        :return: update time stamp
        :rtype: float,dict
        """
        if self.__owner is not vt.ShareType.ALL_WRITE:
            return self.__last_updated[pid]
        else:
            return self.__last_updated

    def set_update(self, update_time_stamp: float, pid: int = -1) -> None:
        """
        setter method for timestamp of last update

        :param update_time_stamp: updated time stamp
        :type update_time_stamp: float

        :param pid: pid of agent to set the update time for
        :type pid: int
        """
        if self.__owner is vt.ShareType.ALL_READ:
            self.__last_updated[pid] = update_time_stamp
        else:
            self.__last_updated = update_time_stamp

    def get_val(self, pid: int = -1) -> tp.Union[
        int, bool, float, list, object, tuple, None]:  # -> Any (not available in python 3.5)
        """
        getter method for value of shared

        :param pid: pid of agent to get the value of
        :type pid: int

        :return: value of shared variable for pid
        :rtype: int,bool,float,list,object,tuple,None
        """
        if self.__owner is vt.ShareType.ALL_READ:
            return self.__value[pid]
        else:
            return self.__value

    def set_val(self, value: tp.Union[int, bool, float, list, object, tuple], pid: int = -1) -> None:
        """
        setter method for value of shared variable
        :param value : value to be set
        :param pid: pid of agent to set the value of
        """
        if self.__owner is vt.ShareType.ALL_READ:
            self.__value[pid] = value
        else:
            self.__value = value
