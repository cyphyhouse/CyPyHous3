# Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

from abc import ABC, abstractmethod
from threading import Thread, Event


class MutexHandler(Thread, ABC):

    def __init__(self):
        super(MutexHandler, self).__init__()
        self.__stop_event = Event()

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

    @abstractmethod
    def run(self):
        pass
