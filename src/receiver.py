import socket, threading
from message import to_msg
import msgpack
import messageHandler
from gvh import Gvh


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
        self.__agent_gvh = Gvh(0)
        self.__ip = ip
        self.__port = port
        self.__stop_event = threading.Event()


    def stop(self):  # -> NoReturn:
        """
         a flag to set to to safely exit the thread
        :return:
        """
        print("stopping receiver")
        self.__stop_event.set()

    def stopped(self):  # -> NoReturn:
        """
        set the stop flag
        :return:
        """
        return self.__stop_event.is_set()

    @property
    def agent_gvh(self):
        return self.__agent_gvh

    @agent_gvh.setter
    def agent_gvh(self, agent_gvh:Gvh):
        self.__agent_gvh = agent_gvh

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

    def recv(self):
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        server_sock.bind((self.ip, self.port))
        data, addr = server_sock.recvfrom(1024)
        server_sock.close()
        return to_msg(msgpack.unpackb(data).decode()).content

    def run(self):
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        server_sock.bind((self.ip, self.port))

        while not self.stopped():
            data, addr = server_sock.recvfrom(1024)
            # TODO Writing to DSM instead of printing
            msg = to_msg(msgpack.unpackb(data).decode())
            print(msg.sender)
            messageHandler.message_handler[msg.m_type](msg,self.agent_gvh)


            #print("Message: ", to_msg(msgpack.unpackb(data).decode()).content)
        print("here")
        server_sock.close()
