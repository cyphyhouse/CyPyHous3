# Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

import time
from typing import Union

from src.harness.commHandler import CommHandler
from src.harness.msg_handle import mutex_grant_create, mutex_release_create, mutex_request_create
from src.objects.abstract.mutex import Mutex


class BaseMutex(Mutex):
    """
    __mutex_id : integer id of mutex
    __mutex_request list : list of agent pids who have requested access to this mutex
    __mutex_holder : agent holding current mutex
    __ip_port_list : list of ports in case sending to same machine. else broadcast to self commhandler receiver port.
    __agent_comm_handler : "reference" to agent comm handler
    """

    def __init__(self, mutex_id: int):
        """
        base init method for mutex
        :param mutex_id: mutex id
        """
        super(BaseMutex, self).__init__()
        self.__mutex_id = mutex_id
        self.__mutex_request_list = []
        self.__mutex_holder = None
        self.__agent_comm_handler = None

    # ------------ MEMBER ACCESS METHODS --------------

    @property
    def mutex_id(self) -> int:
        return self.__mutex_id

    @property
    def mutex_request_list(self) -> list:
        return self.__mutex_request_list

    @property
    def mutex_holder(self) -> Union[None, int]:
        return self.__mutex_holder

    @property
    def agent_comm_handler(self) -> CommHandler:
        return self.__agent_comm_handler

    @mutex_id.setter
    def mutex_id(self, mutex_id: int) -> None:
        self.__mutex_id = mutex_id

    @mutex_holder.setter
    def mutex_holder(self, mutex_holder: Union[int, None]) -> None:
        self.__mutex_holder = mutex_holder

    @mutex_request_list.setter
    def mutex_request_list(self, mutex_request_list: list) -> None:
        self.__mutex_request_list = mutex_request_list

    @agent_comm_handler.setter
    def agent_comm_handler(self, agent_comm_handler: CommHandler) -> None:
        self.__agent_comm_handler = agent_comm_handler

    # ------------ MUTEX HANDLING METHODS --------------

    def request_mutex(self, req_num: int) -> None:
        """
        method to request mutex
        :param req_num: request number
        :type req_num: int
        """
        msg = mutex_request_create(self.mutex_id, req_num, self.agent_comm_handler.agent_gvh.pid,
                                   self.agent_comm_handler.agent_gvh.round_num)
        # print(self.ip_port_list)
        self.agent_comm_handler.agent_gvh.send(msg)

    def grant_mutex(self, mutexnum: int) -> None:
        """
        method to grant mutex, if leader
        :return:
        """
        if self.mutex_holder is None and len(self.mutex_request_list) is not 0:
            agent_id = self.mutex_request_list[0][0]
            self.mutex_holder = agent_id
            self.mutex_request_list = self.mutex_request_list[1:]
            msg = mutex_grant_create(self.mutex_id, agent_id, self.agent_comm_handler.agent_gvh.pid, mutexnum,
                                     time.time())
            self.agent_comm_handler.agent_gvh.send(msg)
        else:
            pass

    def release_mutex(self) -> None:
        """
        method to release mutex
        """
        msg = mutex_release_create(self.mutex_id, self.agent_comm_handler.agent_gvh.pid,
                                   self.agent_comm_handler.agent_gvh.round_num)
        self.agent_comm_handler.agent_gvh.send(msg)
