import time
from typing import Union

from src.config.configs import AgentConfig, MoatConfig
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
        self.__dsm = []
        self.__synchronizer = None
        self.__mutex_handler = a.mutex_handler
        self.__window = 1
        if a.moat_class is not None:
            self.__moat = a.moat_class(m)
        else:
            self.__moat = None
        self.req_nums = {}
        self.ack_nums = {}
        self.init = False
        self.init_counter = []
        if self.__is_leader:
            self.init_counter.append(a.pid)
        self.start_time = 0.0
        self.round_num = 0
        self.update_round = False
        self.round_counter = []

        self.stop_counter = []



    @property
    def rport(self):
        """
        getter method for rport
        :return:
        """
        return self.__rport

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

    @property
    def dsm(self) -> list:
        """
        getter method for the distributed shared memory
        :return:
        """
        return self.__dsm

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
        msg = dsm_update_create(self.pid, a, a.owner, -1)
        if not self.__port_list == []:
            for port in self.__port_list:
                send(msg, AgentConfig.BROADCAST_ADDR, port, self.__window)
        else:
            send(msg, AgentConfig.BROADCAST_ADDR, self.__rport,self.__window)

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
                    send(msg, AgentConfig.BROADCAST_ADDR, port,self.__window)
            else:
                send(msg, AgentConfig.BROADCAST_ADDR, self.__rport,self.__window)

    @property
    def port_list(self):
        """
        getter method for the list of ports on the same machine.
        :return:
        """
        return self.__port_list

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

    @property
    def is_leader(self) -> bool:
        """
        getter method for whether the gvh is of the leader agent .
        :return:
        """
        return self.__is_leader

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

    def add_recv_msg(self, msg):
        """
        add received message to list of recvd messages.
        :param msg:
        :return:
        """
        self.__recv_msg_list.append(msg)

    @property
    def msg_list(self) -> list:
        """

        :return:
        """
        return self.__msg_list

    @property
    def synchronizer(self) -> Synchronizer:
        """
        getter method for synchronizer
        :return: synchronizer object of the current agent
        """
        return self.__synchronizer

    @property
    def pid(self) -> int:
        """
        getter method for pid
        :return: integer pid of the current agent
        """
        return self.__pid

    @property
    def participants(self):
        """
        getter method for number of agents in the system
        :return: integer number of participants
        """
        return self.__participants

    def add_msg(self, msg: Message) -> None:
        """
        add message to list
        :param msg:
        :return:
        """
        self.__msg_list.append(msg)

    def flush_msgs(self) -> None:
        """

        :param msg_list:
        :return:
        """
        for msg in self.msg_list:
            if not self.__port_list == []:
                for port in self.__port_list:
                    send(msg, AgentConfig.BROADCAST_ADDR, port)
            else:
                send(msg, AgentConfig.BROADCAST_ADDR, self.__rport)
        self.__msg_list = []


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
