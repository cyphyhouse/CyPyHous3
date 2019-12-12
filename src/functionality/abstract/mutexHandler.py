# Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

import abc
import threading


class MutexHandler(threading.Thread, abc.ABC):
    """
    abstract mutual exclusion handler class
    """

    def __init__(self):
        super(MutexHandler, self).__init__()
        self.__stop_event = threading.Event()

    def stop(self) -> None:
        """
         a flag to set to to safely exit the thread
        :return: None
        """
        self.__stop_event.set()

    def stopped(self) -> bool:
        """
        set the stop flag
        :return: True if stop event is set, False otherwise
        """
        return self.__stop_event.is_set()

    @abc.abstractmethod
    def grant_available_mutexes(self, *args, **kwargs) -> None:
        """
        abstract method to grant available mutexes
        """
        pass

    @abc.abstractmethod
    def run(self):
        pass
