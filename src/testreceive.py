import pickle
import socket
from threading import Thread, Event

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

    def __init__(self, ip: str, r_port: int, timeout: float = 10.0, retries: int = RETRY_VAL):
        """
        init method for receiver object thread
        :param ip:
        :param r_port:
        :param timeout:
        :param retries:
        """
        super(CommHandler, self).__init__()
        self.__ip = ip
        self.__r_port = r_port
        self.__stop_event = Event()
        self.__timeout = timeout
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
                print(msg)
            except OSError:
                self.stop()
        receiver_socket.close()


class CommTimeoutError(Exception):
    pass
