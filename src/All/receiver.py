import socket, threading
#from typing import NoReturn
from message import to_msg
import msgpack


class Receiver(threading.Thread):
    """
    __stop_event: threading.Event
    __ip: str
    __port: int
    """

    """
    TODO: Merge with Sender potentially
    """

    def __init__(self, ip: str, port: int):
        super(Receiver, self).__init__()
        self.__ip = ip
        self.__port = port
        self.__stop_event = threading.Event()
        self.start()

    def stop(self):  # -> NoReturn:
        """
         a flag to set to to safely exit the thread
        :return:
        """
        self.__stop_event.set()

    def stopped(self):  # -> NoReturn:
        """
        set the stop flag
        :return:
        """
        return self.__stop_event.is_set()

    @property
    def ip(self) -> str:
        """
        getter method for ip
        :return: string ip
        """
        return self.__ip

    @ip.setter
    def ip(self, ip: str):  # -> NoReturn:
        """
        setter method for ip
        """
        self.__ip = ip

    @property
    def port(self) -> int:
        """
        getter method for port
        :return: int port
        """
        return self.__port

    @port.setter
    def port(self, port: int):  # -> NoReturn:
        """
        setter method for ip
        """
        self.__port = port

    def run(self):
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        server_sock.bind((self.ip, self.port))
        while not self.stopped():
            data, addr = server_sock.recvfrom(1024)
            print("Message: ", to_msg(msgpack.unpackb(data).decode()).content)
        server_sock.close()
