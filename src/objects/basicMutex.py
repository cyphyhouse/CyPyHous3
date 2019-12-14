# Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

import time
import typing

import src.harness.commHandler as commHandler
import src.harness.msg_create as mc
import src.objects.abstract.mutex as mutex


class BasicMutex(mutex.Mutex):
    """
    Basic mutex type
    __mutex_id : integer id of mutex
    __mutex_request list : list of agents who have requested access to this mutex
    __mutex_holder : agent holding current mutex
    __ip_port_list : list of ports in case sending to same machine. else broadcast to self comm handler receiver port.
    __agent_comm_handler : "reference" to agent comm handler
    """

    def __init__(self, mutex_id: str):
        """
        :param mutex_id: mutex id
        :type mutex_id: str

        """
        super(BasicMutex, self).__init__()
        self.__mutex_id = mutex_id
        self.__mutex_request_list = []
        self.__mutex_holder = None
        self.__agent_comm_handler = None

    # ------------ MEMBER ACCESS METHODS --------------

    @property
    def mutex_id(self) -> str:
        return self.__mutex_id

    @property
    def mutex_request_list(self) -> list:
        return self.__mutex_request_list

    @property
    def mutex_holder(self) -> typing.Union[None, int]:
        return self.__mutex_holder

    @property
    def agent_comm_handler(self) -> commHandler.CommHandler:
        return self.__agent_comm_handler

    @mutex_id.setter
    def mutex_id(self, mutex_id: str) -> None:
        self.__mutex_id = mutex_id

    @mutex_holder.setter
    def mutex_holder(self, mutex_holder: typing.Union[int, None]) -> None:
        self.__mutex_holder = mutex_holder

    @mutex_request_list.setter
    def mutex_request_list(self, mutex_request_list: list) -> None:
        self.__mutex_request_list = mutex_request_list

    @agent_comm_handler.setter
    def agent_comm_handler(self, agent_comm_handler: commHandler.CommHandler) -> None:
        self.__agent_comm_handler = agent_comm_handler

    # ------------ MUTEX HANDLING METHODS --------------

    def request_mutex(self, req_num: int) -> None:
        """
        method to request mutex

        :param req_num: request number
        :type req_num: int
        """
        msg = mc.mutex_request_create(self.__mutex_id, req_num, self.__agent_comm_handler.agent_gvh.pid,
                                      self.__agent_comm_handler.agent_gvh.round_num)
        self.__agent_comm_handler.agent_gvh.send(msg)

    def grant_mutex(self, mutex_num: int) -> None:
        """
        method to grant mutex, if leader
        """
        if self.__mutex_holder is None and len(self.__mutex_request_list) is not 0:
            agent_id = self.__mutex_request_list[0][0]
            self.__mutex_holder = agent_id
            self.__mutex_request_list = self.__mutex_request_list[1:]
            msg = mc.mutex_grant_create(self.__mutex_id, agent_id, self.__agent_comm_handler.agent_gvh.pid, mutex_num,
                                        time.time())
            self.__agent_comm_handler.agent_gvh.send(msg)
        else:
            pass

    def release_mutex(self) -> None:
        """
        method to release mutex
        """
        msg = mc.mutex_release_create(self.__mutex_id, self.__agent_comm_handler.agent_gvh.pid,
                                      self.__agent_comm_handler.agent_gvh.round_num)
        self.__agent_comm_handler.agent_gvh.send(msg)
