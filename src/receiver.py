import socket, threading
from typing import NoReturn
from message import *
import msgpack


class Receiver(threading.Thread):
    __stop_event: threading.Event
    __ip: str
    __port: int

    def __init__(self, ip: str, port: int):
        super(Receiver, self).__init__()
        self.__ip = ip
        self.__port = port
        self.__stop_event = threading.Event()
        self.start()

    def stop(self) -> NoReturn:
        """
         a flag to set to to safely exit the thread
        :return:
        """
        self.__stop_event.set()

    def stopped(self) -> NoReturn:
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
    def ip(self, ip: str) -> NoReturn:
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
    def port(self, port: int) -> NoReturn:
        """
        setter method for ip
        """
        self.__port = port

    def run(self):
        #UDP_IP_ADDRESS = "127.0.0.1"
        #UDP_PORT_NO = 6794
        serverSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        serverSock.bind((self.ip, self.port))
        while not self.stopped():
            data, addr = serverSock.recvfrom(1024)
            print("Message: ", toMsg(msgpack.unpackb(data).decode()).content)
            #self.stop()
        serverSock.close()
