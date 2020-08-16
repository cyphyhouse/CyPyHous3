import pickle
import select
import socket
from threading import Thread, Event
from queue import Queue

from src.config.configs import AgentConfig

# TODO: move to a base config file.
RETRY_VAL = 10


class CommHandler(Thread):
    """
    communication handler object for the agent application thread.

    __ip : ip of the socket to receive messages on
    __r_port : receive messages on this port
    __timeout : timeout to try receiving a message
    __retries : retries
    """

    def __init__(self, a: AgentConfig, timeout: float = 100.0, retries: int = RETRY_VAL):
        """
        init method for receiver object thread
        :param ip:
        :param r_port:
        :param timeout:
        :param retries:
        """
        super(CommHandler, self).__init__()
        self.__pid = a.pid
        self.__is_leader = a.is_leader
        self.__ip = a.rip
        self.__r_port = a.rport
        self.__stop_event = Event()
        self.__timeout = timeout
        self.receiver_socket = None
        self.__recv_msg_queue = Queue()

    @property
    def timeout(self) -> float:
        """
        getter method of timeout
        :return:
        """
        return self.__timeout

    @property
    def pid(self) -> int:
        """
        getter method for pid
        :return: associated pid
        """
        return self.__pid

    @property
    def is_leader(self) -> bool:
        """
        getter method for is_leader
        :return: associated is_leader
        """
        return self.__is_leader

    @property
    def r_port(self) -> int:
        """
        getter method for the receiver port
        :return: integer port
        """
        return self.__r_port

    @property
    def ip(self) -> str:
        """
        getter method for sender ip
        :return: str ip
        """
        return self.__ip

    @property
    def recv_msg_queue(self) -> Queue:
        """
        getter method for the queue of received messages
        :return: the queue of received messages
        """
        return self.__recv_msg_queue

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

    def __check_receiving_buffer(self, wait_time=0.001) -> bool:
        """
        Check if there are messages in the buffer
        :return: True if there are messages
        """
        r, _, _ = select.select([self.receiver_socket], [], [], wait_time)
        return bool(r)

    def run(self):
        """
        create receiver socket, receive messages.
        :return:
        """
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as self.receiver_socket:
            self.receiver_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

            try:
                self.receiver_socket.bind((self.ip, self.r_port))
                self.receiver_socket.settimeout(self.timeout)
                while not self.stopped():
                    if not self.__check_receiving_buffer(0.01):
                        continue
                    data, addr = self.receiver_socket.recvfrom(8192)
                    #print("packet length",len(data))
                    msg = pickle.loads(data)
                    self.__recv_msg_queue.put(msg)
            except socket.timeout:
                print("agent", self.__pid, "timed out")
                self.stop()
            except OSError as e:
                print(e)
                print("unexpected os error on agent", self.__pid)
