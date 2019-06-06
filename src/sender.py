import socket
# from typing import NoReturn

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
        str_message = str(message)
        clientSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        clientSock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        clientSock.sendto(msgpack.packb(str_message), (self.__broadcast_ip, self.port))
        #clientSock.sendto(msgpack.packb(str_message), (self.ip, self.port))
