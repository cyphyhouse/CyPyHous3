# Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

import pickle
import socket
import time
import typing as tp
import threading

import src.config.configs as configs
import src.harness.gvh as gvh
import src.harness.msg_handle as msg_handle


class CommHandler(threading.Thread):
    """
    communication handler object for the agent application thread.

    __ip : ip of the socket to receive messages on
    __r_port : listener port
    __stop_event : Event indicating status of thread
    __msgs : list of messages
    __agent_gvh : associated agent gvh .
    __r_port : receive messages on this port
    __timeout : timeout to try receiving a message
    """

    def __init__(self, a: configs.AgentConfig, agent_gvh: tp.Union[gvh.Gvh, None] = None, timeout: float = 100.0):
        """
        :param a: agent configuration
        :type a: AgentConfig

        :param agent_gvh: agent gvh
        :type agent_gvh: Gvh

        :param timeout: receiver port timeout
        :type timeout:float
        """
        super(CommHandler, self).__init__()
        self.__ip = a.rip
        self.__r_port = a.r_port
        self.__agent_gvh = agent_gvh
        self.__stop_event = threading.Event()
        self.__timeout = timeout
        self.__msgs = []
        self.receiver_socket = None

        self.start()

    # ------------ MEMBER ACCESS METHODS --------------

    @property
    def timeout(self) -> float:
        return self.__timeout

    @timeout.setter
    def timeout(self, timeout: float) -> None:
        self.__timeout = timeout

    @property
    def agent_gvh(self) -> gvh.Gvh:
        return self.__agent_gvh

    @agent_gvh.setter
    def agent_gvh(self, agent_gvh: gvh.Gvh) -> None:
        self.__agent_gvh = agent_gvh

    @property
    def r_port(self) -> int:
        return self.__r_port

    @r_port.setter
    def r_port(self, r_port: int) -> None:
        self.__r_port = r_port

    @property
    def ip(self) -> str:
        return self.__ip

    @ip.setter
    def ip(self, ip: str) -> None:
        self.__ip = ip

    # ------------ METHODS FOR A GRACEFUL EXIT --------------

    def stop(self) -> None:
        """
         a flag to set to to safely exit the thread
        """
        if self.agent_gvh is not None:
            self.agent_gvh.is_alive = False

        self.__stop_event.set()
        from src.functionality.comm_funcs import send
        from src.harness.msg_create import stop_comm_msg_create
        msg = stop_comm_msg_create(self.agent_gvh.pid, time.time())
        send(msg, self.ip, self.r_port)

    def stopped(self) -> bool:
        """
        set the stop flag

        :return: true if stop event is set, false otherwise
        :rtype: bool
        """
        return self.__stop_event.is_set()

    # ------------ COMMUNICATION HANDLING THREAD BEHAVIOR --------------

    def run(self):
        """
        create receiver socket, receive messages.
        """
        udp_max = 2 ** 16 - 1

        self.receiver_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.receiver_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        try:
            self.receiver_socket.bind((self.ip, self.r_port))
            self.receiver_socket.settimeout(self.timeout)
        except OSError:
            print("perhaps already created socket")

        while not self.stopped():
            try:
                data, addr = self.receiver_socket.recvfrom(udp_max)
                msg = pickle.loads(data)
                self.agent_gvh.add_recv_msg(msg)
            except socket.timeout:
                print("agent", self.agent_gvh.pid, "timed out")
                self.stop()
            except OSError:
                print("unexpected os error on agent", self.agent_gvh.pid)
        try:
            self.receiver_socket.close()
        except OSError:
            print("maybe socket already closed")

    def handle_msgs(self) -> None:
        """
        handle messages method to call message_handle functions. Must be outside the run so that it can be called from
        gvh or agent thread, to avoid blocking receive messages.
        """

        current_list = self.agent_gvh.recv_msg_list.copy()

        for msg in current_list:
            if msg.message_type in list(msg_handle.message_handler.keys()):
                msg_handle.message_handler[msg.message_type](msg, self.agent_gvh)
        self.agent_gvh.recv_msg_list = self.agent_gvh.recv_msg_list[len(current_list):]
