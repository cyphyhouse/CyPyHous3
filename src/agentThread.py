import time
from abc import ABC, abstractmethod
from threading import Thread, Event
from typing import Any

import message
import messageHandler
from commHandler import CommHandler
from gvh import Gvh


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
        self.__comm_handler.receiver.agent_gvh = self.__agent_gvh
        self.__pid = agent_gvh.pid
        # TODO: CHANGE THIS TO LEADER ELECTION . setting leader to be 0
        if self.__pid == 0:
            self.__is_leader = True
        else:
            self.__is_leader = False
        self.__stop_event = Event()
        self.__sleep_event = Event()

    @property
    def is_leader(self) -> bool:
        return self.__is_leader

    @is_leader.setter
    def is_leader(self, val: bool) -> None:
        self.__is_leader = val

    @property
    def num_agents(self) -> int:
        """
        getter method for the number of agents in the system
        :return:
        """
        return self.agent_gvh.participants

    @num_agents.setter
    def num_agents(self, numagents: int):  # -> NoReturn:
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
    def comm_handler(self, ch: CommHandler):  # -> NoReturn:
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
    def agent_gvh(self, agent_gvh: Gvh):  # -> NoReturn:
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
    def pid(self, id: int):  # -> NoReturn:
        """
        setter method for id
        :param id:
        :return:
        """
        self.__id = id

    def stop(self):  # -> NoReturn:
        """
         a flag to set to to safely exit the thread
        :return:
        """
        self.comm_handler.receiver.stop()
        if self.agent_gvh.moat is not None:
            self.agent_gvh.moat.stop()
        self.__stop_event.set()

    def stopped(self):  # -> NoReturn:
        """
        set the stop flag
        :return:
        """
        return self.__stop_event.is_set()

    def sleep(self):  # -> NoReturn:
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

    def mk_var(self, var_scope, var_type, var_name, var_value=None) -> None:
        """
        method to create variable type in dsm
        :param var_scope: scope of variable
        :param var_type: variable type
        :param var_name: name
        :param var_value: value
        :return: nothing
        """
        self.agent_gvh.mk_var(var_scope, var_type, var_name, var_value)

    def send(self, msg: message.Message) -> None:
        """
        send message
        :param msg:
        :return:
        """
        self.comm_handler.send(msg)

    def put(self, pid: int, var_name: str, val: Any) -> None:
        """

        :param pid:
        :param var_name:
        :param val:
        :return:
        """
        key = self.agent_gvh.agent_dsm.sym_tab[var_name]
        vartype = self.agent_gvh.agent_dsm.type_list[key]
        msg = messageHandler.update_create(vartype, pid, var_name, val, time.time())
        self.comm_handler.send(msg)

    def has_mutex(self, var_name: str) -> bool:
        """

        :param var_name:
        :return:
        """
        return self.agent_gvh.has_mutex(var_name)

    def request_mutex(self, var_name:str) -> None:
        """

        :param var_name:
        :return:
        """
        msg = messageHandler.mutex_request_create(var_name, self.pid, time.time())
        self.send(msg)


    def release_mutex(self, var_name:str) -> None:
        """

        :param var_name:
        :return:
        """
        msg = messageHandler.mutex_release_create(var_name, self.pid, time.time())
        self.send(msg)


    def flush_msgs(self):
        msg_list = self.agent_gvh.msg_list
        self.agent_gvh.msg_list = []
        self.comm_handler.flush_msgs(msg_list)


    @abstractmethod
    def run(self) -> None:
        """
        needs to be implemented for any agenThread
        :return:
        """
        pass
