# Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

import typing as tp

import src.config.configs as configs
import src.functionality.abstract.mutex_handler as mh
import src.functionality.comm_funcs as comm_funcs
import src.harness.msg_create as mc
import src.objects.dsm_allread as dsm_ar
import src.objects.dsm_allwrite as dsm_aw
import src.objects.message as message


class gvh(object):
    """
    global variable holder for the agent, this contains the worldview of each agent
    handles distributed shared memory, motion automaton for the object, and any other
    sensor-actuator modules used by the agent in the application.

    __pid : integer indicating the unique identifier of the agent.
    __n_sys : number of robots in the system.
    __msg_list : list of messages to be processed (sent)
    __recv_msg_list : list of messages received.
    __port_list : list of ports agents are listening on if this is being run on the same machine. if empty, broadcast.
    __is_alive : boolean whether agent is still participating in protocol. can be used to stop safely .
    __is_leader : boolean whether agent is leader
    __mutex_handler : mutual exclusion handler
    __send_ips : list of participating ips with corresponding ports. (default broadcast)
    """

    def __init__(self, a: configs.AgentConfig, m: configs.MoatConfig):
        """
        :param a: agent configuration object
        :param m: motion automaton configuration object
        TODO: change motion automaton to arbitrary module configuration(s)
        """
        self.__pid = a.pid
        self.__n_sys = a.bots
        self.__msg_list = []
        self.__recv_msg_list = []
        self.__port_list = a.plist
        self.__r_port = a.rport
        self.__is_leader = a.is_leader
        self.__is_alive = True
        self.__dsm = None
        self.__mutex_handler = a.mutex_handler
        self.__send_ips = []  # TODO : Allow p2p from configuration.

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
        if self.__moat is not None:
            self.__moat.start()

    def start_mh(self) -> None:
        """
        start mutex handler if any
        """
        if self.__mutex_handler is not None:
            self.__mutex_handler.start()

    # ------------ MEMBER ACCESS METHODS --------------

    @property
    def mutex_handler(self):
        return self.__mutex_handler

    @property
    def is_leader(self) -> bool:
        return self.__is_leader

    @property
    def is_alive(self) -> bool:
        return self.__is_alive

    @property
    def msg_list(self) -> tp.List[message.Message]:
        return self.__msg_list

    @property
    def recv_msg_list(self) -> tp.List[message.Message]:
        return self.__recv_msg_list

    @property
    def r_port(self) -> int:
        return self.__r_port

    @property
    def moat(self):  # -> MotionAutomaton
        return self.__moat

    @property
    def dsm(self) -> dict:
        return self.__dsm

    @property
    def send_ips(self) -> tp.List[tp.Tuple[str, int]]:
        return self.__send_ips

    @property
    def port_list(self) -> tp.List[int]:
        return self.__port_list

    @property
    def pid(self) -> int:
        return self.__pid

    @property
    def n_sys(self) -> int:
        return self.__n_sys

    @is_alive.setter
    def is_alive(self, is_alive: bool) -> None:
        self.__is_alive = is_alive

    @msg_list.setter
    def msg_list(self, msg_list: tp.List[message.Message]) -> None:
        self.__msg_list = msg_list

    @recv_msg_list.setter
    def recv_msg_list(self, recv_msg_list: tp.List[message.Message]) -> None:
        self.__recv_msg_list = recv_msg_list

    @dsm.setter
    def dsm(self, my_dsm: tp.Dict) -> None:
        self.__dsm = my_dsm

    @port_list.setter
    def port_list(self, port_list: tp.List[int]) -> None:
        self.__port_list = port_list

    # ------------ SEND RELATED METHODS --------------

    def send(self, msg: message.Message) -> None:
        """
        send method for gvh, to determine where to send the messages.
        :param msg: message to be sent
        :type msg: Message
        """
        if not self.get_multi_flag():
            comm_funcs.send(*self.mk_single_msg_args(msg))
        else:
            multi_list = self.mk_multi_msg_args(msg)
            for elt in multi_list:
                comm_funcs.send(*elt)

    def get_multi_flag(self) -> bool:
        """
        whether multiple messages need to be sent
        :return: true if multiple messages, false if not
        :rtype: bool
        """
        multi_flag = False

        if len(self.__send_ips) == 0:
            if len(self.__port_list) == 0:
                pass
            else:
                multi_flag = True
        else:
            multi_flag = True

        return multi_flag

    def mk_single_msg_args(self, msg: message.Message) -> tp.List:
        """
        returns arguments to call the send method for a single message
        :param msg: Message to be sent
        :type msg: Message
        :return: list of arguments
        :rtype: list
        """
        return [msg, "<broadcast>", self.__r_port]

    def mk_multi_msg_args(self, msg: message.Message) -> tp.List:
        """
        returns arguments to call the send method for multiple messages
        :param msg: Message to be sent
        :type msg: Message
        :return: list of arguments
        :rtype: list
        """
        multi_msg_args = []
        msg_list = [msg] * len(self.__port_list)
        if len(self.__send_ips) == 0:
            if len(self.__port_list) == 0:
                pass
            else:
                b_list = ["<broadcast>"] * len(self.__port_list)
                multi_msg_args = [list(i) for i in zip(msg_list, b_list, self.__port_list)]
        else:
            ip_list = [m[0] for m in self.__send_ips]
            plist = [m[1] for m in self.__send_ips]
            multi_msg_args = [list(i) for i in zip(msg_list, ip_list, plist)]

        return multi_msg_args

    # ------------ SHARED VARIABLE CREATION --------------

    def create_aw_var(self, var_name: str, data_type: type,
                      value: tp.Union[int, bool, float, list, object, tuple, None]) -> None:
        """
        create shared ALL_WRITE variable
        :param var_name: variable name
        :type var_name: str
        :param data_type: data type of ALL_WRITE variable
        :type data_type: type
        :param value: initial value of ALL_WRITE variable
        :type value: int, float, object, list, tuple, None
        """
        a = dsm_aw.dsmAllWrite(var_name, data_type, value)
        if self.__dsm is None:
            self.__dsm = {}
        self.__dsm[var_name] = a

    def create_ar_var(self, var_name: str, data_type: type,
                      value: tp.Union[int, bool, float, list, object, tuple, None]) -> None:
        """
        create shared ALL_READ variable
        :param var_name: variable name
        :type var_name: str
        :param data_type: data type of ALL_READ variable
        :type data_type: type
        :param value: initial value of ALL_READ variable
        :type value: int, float, object, list, tuple, None
        """
        a = dsm_ar.dsmAllRead(var_name, data_type, self.__n_sys, self.__pid, value)
        if self.__dsm is None:
            self.__dsm = {}
        self.__dsm[var_name] = a
        msg = mc.dsm_update_create(self.__pid, a, -1)
        self.send(msg)

    # ------------ SHARED VARIABLE ACCESS METHODS --------------

    def get(self, var_name: str, pid: tp.Union[int, None] = None) -> tp.Union[int, bool, float, list, object, tuple]:
        """
        getter method for shared variables
        :param var_name: variable name
        :type var_name: str
        :param pid: index of agent (None or -1 if ALL_WRITE) updating the variable
        :type pid: int, None
        :return: value of variable
        :rtype: int, float, list, object, tuple, None
        """

        if pid is None:
            pid = -1
        try:
            var = self.__dsm[var_name]
            return var.get_val(pid)
        except IndexError:
            print("trying to read an ALL_READ variable with invalid pid, returning None")
            return None

    def put(self, var_name: str, value: tp.Union[int, bool, float, list, object, tuple],
            pid: tp.Union[int, None] = None) -> None:
        """
        setter method for shared variables
        :param var_name: variable name
        :type var_name: str
        :param value: value of variable
        :type value: int, float, list, object, tuple, None
        :param pid: index of agent (None or -1 if ALL_WRITE) updating the variable
        :type pid: int, None
        """

        if pid is None:
            pid = -1
        var = self.__dsm[var_name]
        var.set_val(value, pid)
        msg = mc.dsm_update_create(self.__pid, var, self.round_num)
        self.send(msg)

    # ------------ MESSAGE PROCESSING METHODS --------------

    def grant_available_mutexes(self) -> None:
        """
        method to grant available mutexes by the leader.
        """
        if self.__mutex_handler is not None:
            self.mutex_handler.grant_available_mutexes()

    def add_recv_msg(self, msg: message.Message) -> None:
        """
        add received message to list of received messages.
        :param msg: message to be added
        :type msg: Message
        """
        self.__recv_msg_list.append(msg)

    def add_msg(self, msg: message.Message) -> None:
        """
        add message to list
        :param msg: message to be added
        :type msg: Message
        """
        self.__msg_list.append(msg)

    def flush_msgs(self) -> None:
        """
        method to send all unsent messages
        """
        for msg in self.msg_list:
            self.send(msg)
        self.__msg_list = []
