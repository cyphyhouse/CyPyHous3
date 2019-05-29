from abc import ABC, abstractmethod
from threading import Thread, Event
from typing import NoReturn

from gvh import Gvh


class AgentThread(ABC, Thread):

    def __init__(self, agentgvh: Gvh):
        __gvh: Gvh
        __pid: int
        __stop_event: Event
        __sleep_event: Event
        """
        abstract object for each agent thread.
        :param gvh: the global variable holder. Holds agent specific info and agent worldview
        """
        super(AgentThread, self).__init__()
        self.__agentGvh = agentgvh
        self.__pid = agentgvh.pid
        self.__stop_event = Event()
        self.__sleep_event = Event()

    @property
    def numAgents(self) -> int:
        """
        getter method for the number of agents in the system
        :return:
        """
        return self.agentGvh.participants

    @numAgents.setter
    def numAgents(self, numagents: int) -> NoReturn:
        """
        setter method for number of agents in the system . may not ever be used
        """
        self.agentGvh.participants = numagents

    @property
    def agentGvh(self) -> Gvh:
        """
        getter method for the gvh
        :return:
        """
        return self.__agentGvh

    @agentGvh.setter
    def agentGvh(self, agentgvh: Gvh) -> NoReturn:
        """
        This may be never used
        :param agentGvh:
        :return:
        """
        self.__agentgvh = agentgvh

    @property
    def pid(self) -> int:
        """
        getter method for the unique numerical identifier
        :return:
        """
        return self.__pid

    @pid.setter
    def pid(self, id: int) -> NoReturn:
        """
        setter method for id
        :param id:
        :return:
        """
        self.__id = id

    def stop(self) -> NoReturn:
        """
         a flag to set to to safely exit the thread
        :return:
        """
        self.__stop_event.set()

    def stopped(self) -> NoReturn:
        """
        set the stop flag
        :return:
        """
        return self.__stop_event.is_set()

    def sleep(self) -> bool:
        """
        allows sleeping
        :return:
        """
        self.__sleep_event.set()

    def is_sleeping(self) -> bool:
        """
        sets the sleeping flag
        :return:
        """
        return self.__sleep_event.is_set()

    @abstractmethod
    def run(self) -> NoReturn:
        """
        needs to be implemented for any agenThread
        :return:
        """
        pass
