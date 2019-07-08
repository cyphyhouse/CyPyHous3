import pickle
import socket
import time
from typing import Union
from dsm import dsmvar
from message import Message
from mutex_handler import BaseMutexHandler
from synchronizer import Synchronizer


class Gvh(object):
    """
    global variable holder for the agent, this contains the worldview of each agent
    handles distributed shared memory, motion automaton for the object, and any other
    sensor-actuator modules used by the agent in the application.

    __pid : integer indicating the unique identifier of the agent.
    __participants : number of participants in the system.
    __synchronizer : barrier synchronization object. Any implementation must implement provided abstract class
    __msg_list : list of messages to be processed
    __recv_msg_list : list of messages received.
    __port_list : list of ports agents are listening on if this is being run on the same machine. if empty, broadcast.
    __is_alive : boolean whether agent is still participating in protocol. can be used to stop safely .
    __is_leader : boolean whether agent is leader
    __mutex_handler : mutual exclusion handler

    """

    def __init__(self, pid: int, participants: int, bot_name: str = 'cyphyhousecopter', mutex_handler=None):
        """
        init method for global variable holder object
        :param pid: integer pid
        :param participants: integer number of participants
        :param mutex_handler : mutual exclusion handler

        """
        self.msg_seq_num = 0
        self.__pid = pid
        self.__participants = participants
        self.__bot_name = bot_name
        self.__msg_list = []
        self.__recv_msg_list = []
        self.__port_list = []
        self.__is_leader = False
        self.__is_alive = True

        # self.__mutex = None
        self.__dsm = None
        self.__synchronizer = None
        self.__mutex_handler = mutex_handler
        self.__moat = None

    def start_moat(self):
        try:
            self.moat.start()
        except:
            print("error starting motion automaton, maybe you don't have ros")

    @property
    def bot_name(self):
        return self.__bot_name

    @bot_name.setter
    def bot_name(self, bot_name):
        self.__bot_name = bot_name

    @property
    def moat(self) :
        """
        getter method for motionAutomaton
        :return:
        """
        return self.__moat

    @moat.setter
    def moat(self, moat):  # -> NoReturn:
        """
        setter method for moAT
        :param moat: motionautomaton
        :return:
        """
        self.__moat = moat

    @property
    def dsm(self) -> list:
        """
        getter method for the distributed shared memory
        :return:
        """
        return self.__dsm

    @dsm.setter
    def dsm(self, dsm: list) -> None:
        """
        setter method for dsm of the current gvh .
        :param dsm:
        :return:
        """
        self.__dsm = dsm

    def create_aw_var(self, varname: str, dtype: type, value: Union[int, bool, float, list, object, tuple]) -> None:
        """
        method to create allwrite variable
        :param varname:
        :param dtype:
        :param value:
        :return:
        """
        a = dsmvar(varname, dtype, self.participants, self.pid, value)
        if self.__dsm is None:
            self.__dsm = []
        self.__dsm.append(a)

    def create_ar_var(self, varname: str, dtype: type, value: Union[int, bool, float, list, object, tuple]) -> None:
        """
        method to create allread variable
        :param varname:
        :param dtype:
        :param value:
        :return:
        """
        a = dsmvar(varname, dtype, self.participants, self.pid, value, 1)
        if self.__dsm is None:
            self.__dsm = []
        self.__dsm.append(a)

    def get(self, varname: str, pid: int = -1) -> Union[int, bool, float, list, object, tuple]:
        """
        getter method for dsm variable value (from local shared memory)
        :param varname:
        :param pid:
        :return:
        """
        for var in self.__dsm:
            if var.name == varname:
                if var.owner == 0:
                    return var.value
                else:
                    if pid == -1:
                        return var.value[self.pid]
                    return var.value[pid]

    def put(self, varname: str, value: Union[int, bool, float, list, object, tuple], pid: int = -1) -> None:
        """
        update method for distributed shared memory variable value
        :param varname:
        :param value:
        :param pid:
        :return:
        """
        for var in self.__dsm:
            if var.name == varname and var.owner == 0:
                var.value = value

            elif var.name == varname and var.owner == 1:
                if pid == -1:
                    var.value[self.pid] = value
                var.value[pid] = value

            msg = dsm_update_create(self.pid, var, var.owner, time.time())
            for port in self.__port_list:
                send(msg, "", port)
                send(msg, "192.168.1.255", port)

    @property
    def port_list(self):
        """
        getter method for the list of ports on the same machine.
        :return:
        """
        return self.__port_list

    @port_list.setter
    def port_list(self, port_list: list):
        """
        set the port list of the same machine.
        :param port_list:
        :return:
        """
        self.__port_list = port_list

    def grant_available_mutexes(self):
        """
        method to grant available mutexes by the leader.
        :return:
        """
        if self.mutex_handler is None:
            pass
        else:
            self.mutex_handler.grant_available_mutexes()

    @property
    def mutex_handler(self):
        """
        getter method for mutex handler.
        :return:
        """
        return self.__mutex_handler

    @mutex_handler.setter
    def mutex_handler(self, mutex_handler: BaseMutexHandler):
        """
        setter method for mutex handler
        :param mutex_handler:
        :return:
        """
        self.__mutex_handler = mutex_handler

    @property
    def is_leader(self) -> bool:
        """
        getter method for whether the gvh is of the leader agent .
        :return:
        """
        return self.__is_leader

    @is_leader.setter
    def is_leader(self, leader: bool) -> None:
        """
        setter method for leader
        :param leader:
        :return:
        """
        self.__is_leader = leader

    @property
    def is_alive(self) -> bool:
        """
        getter method for liveness
        :return:
        """
        return self.__is_alive

    @is_alive.setter
    def is_alive(self, liveness: bool) -> None:
        """
        setter method for liveness
        :param liveness:
        :return:
        """
        self.__is_alive = liveness

    @property
    def recv_msg_list(self) -> list:
        """
        might get rid of this, tracks received messages.
        :return:
        """
        return self.__recv_msg_list

    @recv_msg_list.setter
    def recv_msg_list(self, recv_msg_list) -> None:
        """
        sets received message list
        :param recv_msg_list:
        :return:
        """
        self.__recv_msg_list = recv_msg_list

    def add_recv_msg(self, msg):
        """
        add received message to list of recvd messages.
        :param msg:
        :return:
        """
        self.recv_msg_list.append(msg)

    @property
    def msg_list(self) -> list:
        """

        :return:
        """
        return self.__msg_list

    @msg_list.setter
    def msg_list(self, msg_list: list) -> None:
        """

        :param msg_list:
        :return:
        """
        self.__msg_list = msg_list

    @property
    def synchronizer(self) -> Synchronizer:
        """
        getter method for synchronizer
        :return: synchronizer object of the current agent
        """
        return self.__synchronizer

    @synchronizer.setter
    def synchronizer(self, synchronizer: Synchronizer) -> None:
        """
        setter method for agent synchronizer object
        :param synchronizer: synchronizer object to be set
        :return: None
        """
        self.__synchronizer = synchronizer

    @property
    def pid(self) -> int:
        """
        getter method for pid
        :return: integer pid of the current agent
        """
        return self.__pid

    @pid.setter
    def pid(self, pid: int) -> None:
        """
        setter method for agent pid
        :param pid: integer pid to be set
        :return: None
        """
        self.__pid = pid

    @property
    def participants(self):
        """
        getter method for number of agents in the system
        :return: integer number of participants
        """
        return self.__participants

    @participants.setter
    def participants(self, participants: int) -> None:
        """
        setter method for number of participants
        :param participants: number of participants to be set
        :return: nothing
        """
        self.__participants = participants

    def add_msg(self, msg: Message) -> None:
        """
        add message to list
        :param msg:
        :return:
        """
        self.msg_list.append(msg)

    def flush_msgs(self) -> None:
        """

        :param msg_list:
        :return:
        """
        # print("sending")
        for msg in self.msg_list:
            # print("sending out msg", self.pid)
            for port in self.__port_list:
                send(msg, "", port)
                send(msg, "192.168.1.255", port)
        self.msg_list = []


def dsm_update_create(pid: int, dsmvar_updated, owner, ts):
    """
    create dsm update message. cant import from message handler because of circular imports.
    :param pid:
    :param dsmvar_updated:
    :param owner:
    :param ts:
    :return:
    """
    return Message(pid, 4, dsmvar_updated, ts)


def send(msg: Message, ip: str, port: int) -> None:
    """
    :param msg: message to be sent
    :param ip: ip to be sent to
    :param port: port to be sent to
    :return:
    """
    client_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    client_sock.sendto(pickle.dumps(msg), (ip, port))
