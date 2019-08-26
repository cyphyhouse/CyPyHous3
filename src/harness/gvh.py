import time
from typing import Union

from src.config.configs import AgentConfig, MoatConfig
from src.functionality.base_mutex_handler import BaseMutexHandler
from src.functionality.comm_funcs import send
from src.functionality.synchronizer import Synchronizer
from src.objects.dsm import DSM
from src.objects.message import Message


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

    def __init__(self, a: AgentConfig, m: MoatConfig):
        """
        init method for global variable holder object
        :param pid: integer pid
        :param participants: integer number of participants
        :param mutex_handler : mutual exclusion handler

        """
        self.__pid = a.pid
        self.__participants = a.bots
        self.__msg_list = []
        self.__recv_msg_list = []
        self.__port_list = a.plist
        self.__rport = a.rport
        self.__is_leader = a.is_leader
        self.__is_alive = True
        self.__dsm = None
        self.__synchronizer = None
        self.__mutex_handler = a.mutex_handler
        if a.moat_class is not None:
            self.__moat = a.moat_class(m)
        else:
            self.__moat = None

        self.init = False
        self.init_counter = []
        if self.is_leader:
            self.init_counter.append(a.pid)

        self.round_num = 0
        self.update_round = False
        self.round_counter = []

        self.stop_counter = []



    def start_moat(self):
        if self.moat is not None:
            self.moat.start()

    def start_mh(self):
        if self.mutex_handler is not None:
            self.mutex_handler.start()



    @property
    def moat(self):
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
        a = DSM(varname, dtype, self.participants, self.pid, value)
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
        a = DSM(varname, dtype, self.participants, self.pid, value, 1)
        if self.__dsm is None:
            self.__dsm = []
        self.__dsm.append(a)
        msg = dsm_update_create(self.pid, a, a.owner, time.time())
        if not self.__port_list == []:
            for port in self.__port_list:
                send(msg, "<broadcast>", port)
        else:
            send(msg, "<broadcast>", self.__rport)

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
                    try:
                        if pid == -1:
                            return var.value
                        return var.value[pid]
                    except IndexError:
                        print("trying to read an allread variable with invalid pid, returning None")
                        return None

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

            msg = dsm_update_create(self.pid, var, var.owner, self.round_num)
            if not self.__port_list == []:
                for port in self.__port_list:
                    send(msg, "<broadcast>", port)
            else:
                send(msg, "<broadcast>", self.__rport)

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
        for msg in self.msg_list:
            if not self.__port_list == []:
                for port in self.__port_list:
                    send(msg, "<broadcast>", port)
            else:
                send(msg, "<broadcast>", self.__rport)
        self.msg_list = []


def dsm_update_create(pid: int, dsmvar_updated, owner, roundnum):
    """
    create dsm update message. cant import from message handler because of circular imports.
    :param pid:
    :param dsmvar_updated:
    :param owner:
    :param ts:
    :return:
    """
    return Message(pid, 4, dsmvar_updated, roundnum)
