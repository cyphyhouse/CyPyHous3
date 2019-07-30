import pickle
import socket
from threading import Thread, Event

from src.config.configs import AgentConfig
from src.harness.gvh import Gvh
from src.harness.message_handler import message_handler

# TODO: move to a base config file.
RETRY_VAL = 10


class CommHandler(Thread):
    """
    communication handler object for the agent application thread.

    __ip : ip of the socket to receive messages on
    __agent_gvh : associated agent gvh .
    __r_port : receive messages on this port
    __timeout : timeout to try receiving a message
    __retries : retries
    """

    def __init__(self, a: AgentConfig, agent_gvh=None, timeout: float = 10.0, retries: int = RETRY_VAL):
        """
        init method for receiver object thread
        :param ip:
        :param r_port:
        :param timeout:
        :param retries:
        """
        super(CommHandler, self).__init__()
        self.__ip = a.rip
        self.__r_port = a.rport
        self.__agent_gvh = agent_gvh
        self.__stop_event = Event()
        self.__timeout = timeout
        self.__msgs = []
        self.start()

    @property
    def timeout(self) -> float:
        """
        getter method of timeout
        :return:
        """
        return self.__timeout

    @timeout.setter
    def timeout(self, timeout: float) -> None:
        """
        setter method for timeout
        :param timeout: float value to be set
        :return:
        """
        self.__timeout = timeout

    @property
    def agent_gvh(self) -> Gvh:
        """
        getter method for agent gvh
        :return: associated agent gvh
        """
        return self.__agent_gvh

    @agent_gvh.setter
    def agent_gvh(self, agent_gvh: Gvh) -> None:
        """
        agent gvh setter method
        :param agent_gvh: agent gvh to be linked
        :return: None
        """
        self.__agent_gvh = agent_gvh

    @property
    def r_port(self) -> int:
        """
        getter method for the receiver port
        :return: integer port
        """
        return self.__r_port

    @r_port.setter
    def r_port(self, r_port: int) -> None:
        """
        setter method for the port
        :param r_port: port number to set
        :return: None
        """
        self.__r_port = r_port

    @property
    def ip(self) -> str:
        """
        getter method for sender ip
        :return: str ip
        """
        return self.__ip

    @ip.setter
    def ip(self, ip: str) -> None:
        """
        setter method for sender ip
        :param ip: string ip
        :return: None
        """
        self.__ip = ip

    def stop(self) -> None:
        """
         a flag to set to to safely exit the thread
        :return: None
        """
        if self.agent_gvh is not None:
            self.agent_gvh.is_alive = False
        self.__stop_event.set()

    def stopped(self) -> bool:
        """
        set the stop flag
        :return: true if stop event is set, false otherwise
        """
        return self.__stop_event.is_set()

    def run(self):
        """
        create receiver socket, receive messages.
        :return:
        """
        receiver_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        receiver_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        try:
            receiver_socket.bind((self.ip, self.r_port))
            receiver_socket.settimeout(self.timeout)
        except OSError:
            print("perhaps already created socket")

        while not self.stopped():
            try:
                data, addr = receiver_socket.recvfrom(4096)
                msg = pickle.loads(data)
                self.agent_gvh.add_recv_msg(msg)
            except socket.timeout:
                print("agent", self.agent_gvh.pid, "timed out")
                self.stop()
            except OSError:
                print("unexpected os error on agent", self.agent_gvh.pid)

        receiver_socket.close()

    def handle_msgs(self) -> None:
        """
        handle messages method to call message_handle functions. Must be outside the run so that it can be called from
        gvh or agent thread, to avoid blocking receive messages.
        TODO: better explanation
        :return: none
        """

        current_list = self.agent_gvh.recv_msg_list.copy()

        for msg in current_list:
            if msg.message_type in list(message_handler.keys()):
                message_handler[msg.message_type](msg, self.agent_gvh)
        self.agent_gvh.recv_msg_list = self.agent_gvh.recv_msg_list[len(current_list):]


class CommTimeoutError(Exception):
    pass
