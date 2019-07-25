import time
from typing import Union

from src.functionality.comm_funcs import send
from src.harness.comm_handler import CommHandler
from src.harness.message_handler import base_mutex_grant_create, mutex_release_create, base_mutex_request_create
from src.objects.mutex import Mutex


class BaseMutex(Mutex):
    """
    __mutex_id : integer id of mutex
    __mutex_request list : list of agent pids who have requested access to this mutex
    __mutex_holder : agent holding current mutex
    __ip_port_list : list of ports in case sending to same machine. else broadcast to self commhandler receiver port.
    __agent_comm_handler : "reference" to agent comm handler
    """

    def __init__(self, mutex_id: int, ip_port_list: list = []):
        """
        base init method for mutex
        :param mutex_id: mutex id
        :param ip_port_list: list of ports in case same machine
        """
        super(BaseMutex, self).__init__()
        self.__mutex_id = mutex_id
        self.__mutex_request_list = []
        self.__mutex_holder = None
        self.__ip_port_list = ip_port_list
        self.__agent_comm_handler = None

    @property
    def mutex_id(self) -> int:
        """
        mutex id getter
        :return:
        """
        return self.__mutex_id

    @property
    def mutex_request_list(self) -> list:
        """
        getter for mutex request list
        :return:
        """
        return self.__mutex_request_list

    @property
    def mutex_holder(self) -> Union[None, int]:
        """
        getter method for the mutex holder.
        :return:
        """
        return self.__mutex_holder

    @property
    def ip_port_list(self) -> list:
        """
        getter method for the list of ports to send messages to.
        :return:
        """
        return self.__ip_port_list

    @property
    def agent_comm_handler(self) -> CommHandler:
        """
        getter method for associated communication handler.
        :return:
        """
        return self.__agent_comm_handler

    @mutex_id.setter
    def mutex_id(self, mutex_id: int) -> None:
        """
        setter method for mutex_id
        :param mutex_id: int mutex id
        :return:
        """
        self.__mutex_id = mutex_id

    @mutex_holder.setter
    def mutex_holder(self, mutex_holder: Union[int, None]) -> None:
        """
        setter method for mutex holder
        :param mutex_holder:
        :return:
        """
        self.__mutex_holder = mutex_holder

    @mutex_request_list.setter
    def mutex_request_list(self, mutex_request_list: list) -> None:
        """
        setter method for mutex request list.
        :param mutex_request_list:
        :return:
        """
        self.__mutex_request_list = mutex_request_list

    @ip_port_list.setter
    def ip_port_list(self, ip_port_list: list) -> None:
        """
        setter method for list of ports.
        :param ip_port_list:
        :return:
        """
        self.__ip_port_list = ip_port_list

    @agent_comm_handler.setter
    def agent_comm_handler(self, agent_comm_handler: CommHandler) -> None:
        """
        setter method for agent communication handler.
        :param agent_comm_handler:
        :return:
        """
        self.__agent_comm_handler = agent_comm_handler

    def request_mutex(self, req_num: int) -> None:
        """
        method to request mutex
        :param req_num: request number
        :return:
        """
        msg = base_mutex_request_create(self.mutex_id, req_num, self.agent_comm_handler.agent_gvh.pid, time.time())
        if self.ip_port_list is not []:
            for port in self.ip_port_list:
                send(msg, '<broadcast>', port)
        else:
            print("sending request here")
            send(msg, '<broadcast>', self.agent_comm_handler.r_port)

    def grant_mutex(self, mutexnum: int) -> None:
        """
        method to grant mutex, if leader
        :return:
        """
        if self.mutex_holder is None and len(self.mutex_request_list) is not 0:
            agent_id = self.mutex_request_list[0][0]
            self.__mutex_holder = agent_id
            self.__mutex_request_list = self.mutex_request_list[1:]
            msg = base_mutex_grant_create(self.mutex_id, agent_id, self.agent_comm_handler.agent_gvh.pid, mutexnum,
                                          time.time())

            if self.ip_port_list is not []:
                for port in self.ip_port_list:
                    send(msg, '<broadcast>', port)
            else:
                send(msg, '<broadcast>', self.agent_comm_handler.r_port)
        else:
            pass

    def release_mutex(self):
        msg = mutex_release_create(self.mutex_id, self.agent_comm_handler.agent_gvh.pid, time.time())
        if self.ip_port_list is not []:
            for port in self.ip_port_list:
                send(msg, '<broadcast>', port)
        else:
            send(msg, '<broadcast>', self.agent_comm_handler.r_port)
