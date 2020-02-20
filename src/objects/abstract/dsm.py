# Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

from abc import ABC, abstractmethod


class dsm(ABC):
    """
    abstract distributed shared memory object
    __name : variable name
    __data_type : data type
    """

    def __init__(self, name: str, data_type: type):
        """

        :param name: variable name
        :type name: str

        :param data_type: variable datatype
        :type data_type: type

        """
        self.__name = name
        self.__data_type = data_type

    def __repr__(self):
        """
        string representation
        """
        return str(self.name)

        # ------------ MEMBER ACCESS METHODS --------------

    @property
    def name(self) -> str:
        return self.__name

    @property
    def data_type(self) -> type:
        return self.__data_type

    # ------------ MEMBER ACCESS METHODS FOR POTENTIALLY LIST ITEMS--------------

    @abstractmethod
    def last_update(self, **kwargs):
        """
        abstract method for timestamp of last update

        """
        pass

    @abstractmethod
    def set_update(self, **kwargs) -> None:
        """
        abstract setter method for timestamp of last update

        """
        pass

    @abstractmethod
    def get_val(self):
        """
        abstract getter method for value of shared
        """
        pass

    @abstractmethod
    def set_val(self, **kwargs) -> None:
        """
        abstract setter method for value of shared variable
        """
        pass
