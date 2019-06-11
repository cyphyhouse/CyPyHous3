import socket

import msgpack

from message import Message


class Sender(object):
    """
    __ip: str
    __port: int
    """

    def __init__(self, ip: str, port: int):
        self.__broadcast_ip = '192.168.1.255'
        self.__ip = ip
        self.__port = port

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
        setter method for port
        """
        self.__port = port

    def send(self, message: Message):  # -> NoReturn:
        """

        :param message: message to be sent
        :return:
        """
        str_message = str(message)
        client_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        client_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        client_sock.sendto(msgpack.packb(str_message), ("192.168.1.255", self.port))
        client_sock.sendto(msgpack.packb(str_message), (self.ip, self.port))
