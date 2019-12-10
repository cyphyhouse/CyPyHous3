# Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

from typing import Union, List, Tuple

from src.config.configs import AgentConfig, MoatConfig
from src.functionality.abstract.mutex_handler import MutexHandler
from src.functionality.comm_funcs import send
from src.harness.msg_create import dsm_update_create
from src.objects.dsm import DSM
from src.objects.message import Message


class Gvh(object):
    """
    global variable holder for the agent, this contains the worldview of each agent
    handles distributed shared memory, motion automaton for the object, and any other
    sensor-actuator modules used by the agent in the application.

    __pid : integer indicating the unique identifier of the agent.
    __nsys : number of robots in the system.
    __msg_list : list of messages to be processed (sent)
    __recv_msg_list : list of messages received.
    __port_list : list of ports agents are listening on if this is being run on the same machine. if empty, broadcast.
    __is_alive : boolean whether agent is still participating in protocol. can be used to stop safely .
    __is_leader : boolean whether agent is leader
    __mutex_handler : mutual exclusion handler
    __send_ips : list of participating ips with corresponding ports. (default broadcast)

    """

    def __init__(self, a: AgentConfig, m: MoatConfig):
        """
        :param a: agent configuration object
        :param m: motion automaton configuration object
        TODO: change motion automaton to arbitrary module configuration(s)
        """
        self.__pid = a.pid
        self.__nsys = a.bots
        self.__msg_list = []
        self.__recv_msg_list = []
        self.__port_list = a.plist
        self.__rport = a.rport
        self.__is_leader = a.is_leader
        self.__is_alive = True
        self.__dsm = None
        self.__mutex_handler = a.mutex_handler
        self.__send_ips = a.send_ips
        if a.moat_class is not None:
            self.__moat = a.moat_class(m)
        else:
            self.__moat = None

        # for barrier synchronization

        self.init = False
        self.init_counter = []  # counter for number of robots initialized. primarily for use by leader.
        if self.is_leader:
            self.init_counter.append(a.pid)

        self.round_num = 0  # counter for round number. Every robot should be * within one round of all others *.
        self.update_round = False
        self.round_counter = []  # counter for number of robots at end of round. primarily for use by leader.

        self.stop_counter = []

    # ------------ ASSOCIATED THREAD START METHODS --------------

    def start_moat(self) -> None:
        """
        start motion automaton if any.
        """
        if self.moat is not None:
            self.moat.start()

    def start_mh(self) -> None:
        """
        start mutex handler if any
        """
        if self.mutex_handler is not None:
            self.mutex_handler.start()

    # ------------ MEMBER ACCESS METHODS --------------

    @property
    def mutex_handler(self):
        """
        getter method for mutex handler.
        :return:
        """
        return self.__mutex_handler

    @mutex_handler.setter
    def mutex_handler(self, mutex_handler: MutexHandler):
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
    def msg_list(self) -> List[Message]:
        """
        getter method for list of messages
        :return:
        """
        return self.__msg_list

    @msg_list.setter
    def msg_list(self, msg_list: List[Message]) -> None:
        """
        sets message list
        :param msg_list: list
        :return:
        """
        self.__msg_list = msg_list

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

    @property
    def rport(self) -> int:
        """
        getter method for listener port
        :return: port number
        """
        return self.__rport

    @rport.setter
    def rport(self, rport: int) -> None:
        """
        setter method for listener port
        :param rport: integer port number
        :return: Nothing
        """
        self.__rport = rport

    @property
    def moat(self):  # -> MotionAutomaton
        """
        getter method for motionAutomaton
        :return:
        """
        return self.__moat

    @moat.setter
    def moat(self, moat) -> None:
        """
        setter method for motion automaton if any
        :param moat: motionautomaton
        :return:
        """
        self.__moat = moat

    @property
    def dsm(self):
        """
        getter method for the distributed shared memory
        :return: associated dsm object
        """
        return self.__dsm

    @dsm.setter
    def dsm(self, dsm) -> None:
        """
        setter method for dsm of the current gvh .
        :param dsm:
        :return:
        """
        self.__dsm = dsm

    @property
    def send_ips(self) -> List[Tuple[str, int]]:
        """
        :return: ips of participants with corresponding ports (default broadcast)
        :rtype: List[Tuple[str,int]]
        """
        return self.__send_ips

    @send_ips.setter
    def send_ips(self, send_ips: List[Tuple[str, int]]) -> None:
        """
        :param send_ips: list of ips with corresponding ports
        :type send_ips: List[Tuple[str,int]]
        """
        self.__send_ips = send_ips

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
    def nsys(self):
        """
        getter method for number of agents in the system
        :return: integer number of nsys
        """
        return self.__nsys

    @nsys.setter
    def nsys(self, nsys: int) -> None:
        """
        setter method for number of robots
        :param nsys: number of robots to be set
        :return: nothing
        """
        self.__nsys = nsys

    # ------------ SEND RELATED METHODS --------------

    def send(self, msg: Message) -> None:
        """
        send method for gvh, to determine where to send the messages.
        :param msg: message to be sent
        :type msg: Message
        """
        if not self.get_multi_flag():
            send(*self.mk_single_msg_args(msg))
        else:
            multilist = self.mk_multi_msg_args(msg)
            for elt in multilist:
                send(*elt)

    def get_multi_flag(self) -> bool:
        """
        whether multiple messages need to be sent
        :return: true if multiple messages, false if not
        :rtype: bool
        """
        multi_flag = False

        if len(self.send_ips) == 0:
            if len(self.port_list) == 0:
                pass
            else:
                multi_flag = True
        else:
            multi_flag = True

        return multi_flag

    def mk_single_msg_args(self, msg: Message) -> list:
        """
        returns arguments to call the send method for a single message
        :param msg: Message to be sent
        :type msg: Message
        :return: list of arguments
        :rtype: list
        """
        return [msg, "<broadcast>", self.rport]

    def mk_multi_msg_args(self, msg: Message) -> list:
        """
        returns arguments to call the send method for multiple messages
        :param msg: Message to be sent
        :type msg: Message
        :return: list of arguments
        :rtype: list
        """
        multi_msg_args = []
        msglist = [msg for i in self.port_list]
        if len(self.send_ips) == 0:
            if len(self.port_list) == 0:
                pass
            else:
                blist = ["<broadcast>" for i in self.port_list]
                multi_msg_args = [list(i) for i in zip(msglist, blist, self.port_list)]
        else:
            iplist = [m[0] for m in self.send_ips]
            plist = [m[1] for m in self.send_ips]
            multi_msg_args = [list(i) for i in zip(msglist, iplist, plist)]

        return multi_msg_args

    # ------------ SHARED VARIABLE CREATION --------------

    def create_aw_var(self, varname: str, dtype: type,
                      value: Union[int, bool, float, list, object, tuple, None]) -> None:
        """
        method to create allwrite variable
        :param varname: variable name
        :param dtype: data type
        :param value: intial value
        """
        a = DSM(varname, dtype, self.nsys, self.pid, value)
        if self.__dsm is None:
            self.__dsm = {}
        self.__dsm[varname] = a

    def create_ar_var(self, varname: str, dtype: type,
                      value: Union[int, bool, float, list, object, tuple, None]) -> None:
        """
        method to create allread variable
        :param varname: variable name
        :param dtype: data type
        :param value: intial value
        """
        a = DSM(varname, dtype, self.nsys, self.pid, value, 1)
        if self.__dsm is None:
            self.__dsm = {}
        self.__dsm[varname] = a
        msg = dsm_update_create(self.pid, a, a.owner, -1)
        self.send(msg)

    # ------------ SHARED VARIABLE ACCESS METHODS --------------

    def get(self, varname: str, pid: Union[int,None] = None) -> Union[int, bool, float, list, object, tuple]:
        """
        getter method for dsm variable value (from local shared memory)
        :param varname: variable name
        :param pid: pid of agent
        :return: value of variable
        """
        if pid is None:
            pid = -1
        try:
            var = self.dsm[varname]
            return var.get_val(pid)
        except IndexError:
            print("trying to read an allread variable with invalid pid, returning None")
            return None

    def put(self, varname: str, value: Union[int, bool, float, list, object, tuple], pid: Union[int,None] = None) -> None:
        """
        update method for distributed shared memory variable value
        :param varname: variable name
        :param pid: pid of agent
        :param value: value of variable
        """
        if pid is None:
            pid = -1
        var = self.dsm[varname]
        var.set_val(value, pid)
        msg = dsm_update_create(self.pid, var, var.owner, self.round_num)
        self.send(msg)

    # ------------ MESSAGE PROCESSING METHODS --------------

    def grant_available_mutexes(self) -> None:
        """
        method to grant available mutexes by the leader.
        """
        if self.mutex_handler is None:
            pass
        else:
            self.mutex_handler.grant_available_mutexes()

    def add_recv_msg(self, msg: Message) -> None:
        """
        add received message to list of recvd messages.
        :param msg: message to be added
        :type msg: Message
        """
        self.recv_msg_list.append(msg)

    def add_msg(self, msg: Message) -> None:
        """
        add message to list
        :param msg: message to be added
        :type msg: Message
        """
        self.msg_list.append(msg)

    def flush_msgs(self) -> None:
        """
        method to send all unsent messages
        :param msg_list: list of messages
        :type msg_list: list
        """
        for msg in self.msg_list:
            self.send(msg)
        self.msg_list = []
