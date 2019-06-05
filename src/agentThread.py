from abc import ABC, abstractmethod
from threading import Thread, Event
#from typing import NoReturn

from gvh import Gvh
from commHandler import CommHandler


class AgentThread(ABC, Thread):

    def __init__(self, agent_gvh: Gvh, comm_handler: CommHandler):
        """
        __gvh: Gvh
        __pid: int
        __stop_event: Event
        __sleep_event: Event
        """

        """
        abstract object for each agent thread.
        :param gvh: the global variable holder. Holds agent specific info and agent worldview
        """
        super(AgentThread, self).__init__()
        self.__agent_gvh = agent_gvh
        self.__comm_handler = comm_handler
        self.__pid = agent_gvh.pid
        self.__stop_event = Event()
        self.__sleep_event = Event()

    @property
    def num_agents(self) -> int:
        """
        getter method for the number of agents in the system
        :return:
        """
        return self.agent_gvh.participants

    @num_agents.setter
    def num_agents(self, numagents: int) : #-> NoReturn:
        """
        setter method for number of agents in the system . may not ever be used
        """
        self.agent_gvh.participants = numagents

    @property
    def comm_handler(self) -> CommHandler:
        """
        getter method for the commhandler
        :return:
        """
        return self.__comm_handler

    @comm_handler.setter
    def comm_handler(self, ch: CommHandler) : #-> NoReturn:
        """
        This may be never used
        :param ch:
        :return:
        """
        self.__comm_handler = ch

    @property
    def agent_gvh(self) -> Gvh:
        """
        getter method for the gvh
        :return:
        """
        return self.__agent_gvh

    @agent_gvh.setter
    def agent_gvh(self, agent_gvh: Gvh) : #-> NoReturn:
        """
        This may be never used
        :param agent_gvh:
        :return:
        """
        self.__agent_gvh = agent_gvh

    @property
    def pid(self) -> int:
        """
        getter method for the unique numerical identifier
        :return:
        """
        return self.__pid

    @pid.setter
    def pid(self, id: int) : #-> NoReturn:
        """
        setter method for id
        :param id:
        :return:
        """
        self.__id = id

    def stop(self) : #-> NoReturn:
        """
         a flag to set to to safely exit the thread
        :return:
        """
        self.__stop_event.set()

    def stopped(self) : #-> NoReturn:
        """
        set the stop flag
        :return:
        """
        return self.__stop_event.is_set()

    def sleep(self) : #-> NoReturn:
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
    def run(self) : #-> NoReturn:
        """
        needs to be implemented for any agenThread
        :return:
        """
        pass
