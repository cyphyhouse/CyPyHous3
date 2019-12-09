# Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

from abc import ABC, abstractmethod


class Mutex(ABC):
    """
    abstract mutual exclusion class
    """

    def __init__(self):
        """
        base init method
        """
        pass

    @abstractmethod
    def request_mutex(self, *args, **kwargs) -> None:
        """
        request mutex abstract method
        """
        pass

    @abstractmethod
    def grant_mutex(self, *args, **kwargs) -> None:
        """
        grant mutex abstract method
        """
        pass

    @abstractmethod
    def release_mutex(self, *args, **kwargs) -> None:
        """
        release mutex abstract method
        """
        pass
