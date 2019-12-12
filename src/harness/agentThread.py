# Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

import signal
import sys
import time
import abc
import threading
import typing as tp

import src.config.configs as configs
import src.datatypes.mutex_handler_types as mh_types
import src.functionality.abstract.mutexHandler as mh
import src.harness.commHandler as ch
import src.harness.msg_create as mc
from src.harness.gvh import Gvh


class AgentThread(abc.ABC, threading.Thread):
    """
    abstract object for each agent thread/application. This contains
    the methods and fields each agent application must implement.

    __agent_gvh : global variable holder object
    __agent_comm_handler : communication handling thread
    __stop_event : stop event for graceful exit
    __do_loop_body : whether or not application loop body should be executed
    __mutex_handler : associated mutex handler
    __locals : local variable map
    __requested_mutexes : map of mutexes to boolean indicating whether mutex has been requested
    __request_nums : request id to differentiate between multiple (separate) requests
    __mutexes : mutex map

    """

    def __init__(self, agent_config: configs.AgentConfig, moat_config: tp.Union[configs.MoatConfig, None]) -> None:
        """

        :param agent_config: agent configuration
        :param moat_config: motion automaton configuration
        TODO: make moat_config a list of arbitrary module configurations
        """
        super(AgentThread, self).__init__()
        self.__agent_gvh = Gvh(agent_config, moat_config)
        self.__agent_comm_handler = ch.CommHandler(agent_config, self.__agent_gvh)
        self.__agent_gvh.start_moat()
        self.__agent_gvh.start_mh()
        self.__stop_event = threading.Event()
        self.__do_loop_body = True
        self.__mutex_handler = self.__agent_gvh.mutex_handler
        self.__locals = {}

        self.__requested_mutexes = {}
        self.__request_nums = {}
        self.__mutexes = {}

        # create a signal handler to handle ctrl + c
        signal.signal(signal.SIGINT, self.signal_handler)
        # create init messages, and keep sending until leader acknowledges, then start app thread.
        # leader only starts once everyone has received an ack.

    # ------------ MEMBER ACCESS METHODS --------------

    @property
    def do_loop_body(self) -> bool:
        return self.__do_loop_body

    @property
    def locals(self):
        return self.__locals

    @property
    def agent_gvh(self) -> Gvh:
        return self.__agent_gvh

    @property
    def agent_comm_handler(self) -> ch.CommHandler:
        return self.__agent_comm_handler

    @property
    def mutex_handler(self) -> mh.MutexHandler:
        return self.__mutex_handler

    @property
    def requested_mutexes(self) -> tp.Dict:
        return self.__requested_mutexes

    @property
    def request_nums(self) -> tp.Dict:
        return self.__request_nums

    @property
    def mutexes(self) -> tp.Dict:
        return self.__mutexes

    @mutexes.setter
    def mutexes(self, mutexes: tp.Dict) -> None:
        self.__mutexes = mutexes

    @request_nums.setter
    def request_nums(self, request_nums: tp.Dict) -> None:
        self.__request_nums = request_nums

    @requested_mutexes.setter
    def requested_mutexes(self, requested_mutexes: tp.Dict) -> None:
        self.__requested_mutexes = requested_mutexes

    @mutex_handler.setter
    def mutex_handler(self, mutex_handler: mh.MutexHandler) -> None:
        self.__mutex_handler = mutex_handler

    @do_loop_body.setter
    def do_loop_body(self, do_loop_body: bool) -> None:
        self.__do_loop_body = do_loop_body

    @locals.setter
    def locals(self, local_dict: tp.Dict):
        self.__locals = local_dict

    @agent_gvh.setter
    def agent_gvh(self, agent_gvh: Gvh) -> None:
        self.__agent_gvh = agent_gvh

    @agent_comm_handler.setter
    def agent_comm_handler(self, agent_comm_handler) -> None:
        self.__agent_comm_handler = agent_comm_handler

    # ------------ SYSTEM PROPERTY ACCESS METHODS --------------

    def pid(self) -> int:
        """
        get the pid of the current agent.
        uses the agent gvh to retrieve it

        :return: pid of the current agent
        :rtype: int
        """
        return self.__agent_gvh.pid

    def num_agents(self) -> int:
        """
        method to return the number of agents in the system

        :return: number of agents
        :rtype: int
        """
        return self.__agent_gvh.n_sys

    def receiver_port(self) -> int:
        """
        gets the port the application is receiving messages on

        :return: receiver port
        :rtype: int
        """
        return self.__agent_comm_handler.r_port

    def receiver_ip(self) -> str:
        """
        method to get the ip of receiver

        :return: ip of receiver
        :rtype: str
        """
        return self.__agent_comm_handler.ip

    # ------------ SHARED VARIABLE CREATION AND UPDATE METHODS --------------

    def create_ar_var(self, name: str, data_type: type,
                      initial_value: tp.Union[int, float, object, list, tuple, None] = None) -> None:
        """
        create shared ALL_READ variable

        :param name: variable name
        :type name: str

        :param data_type: data type of ALL_READ variable
        :type data_type: type

        :param initial_value: initial value of ALL_READ variable
        :type initial_value: int, float, object, list, tuple, None

        """
        self.__agent_gvh.create_ar_var(name, data_type, initial_value)

    def create_aw_var(self, name: str, data_type: type,
                      initial_value: tp.Union[int, float, object, list, tuple, None] = None) -> None:
        """
        create shared ALL_WRITE variable

        :param name: variable name
        :type name: str

        :param data_type: data type of ALL_WRITE variable
        :type data_type: type

        :param initial_value: initial value of ALL_WRITE variable
        :type initial_value: int, float, object, list, tuple, None

        """
        self.__agent_gvh.create_aw_var(name, data_type, initial_value)

    def write_to_shared(self, var_name: str, index: tp.Union[int, None],
                        value: tp.Union[int, float, list, object, tuple, None]) -> None:
        """
        write to shared variables

        :param var_name: variable name
        :type var_name: str

        :param index: index of agent (None or -1 if ALL_WRITE) updating the variable
        :type index: int, None

        :param value: value of variable
        :type value: int, float, list, object, tuple, None

        """
        self.__agent_gvh.put(var_name, value, index)

    def read_from_shared(self, var_name: str, index: tp.Union[int, None]) -> tp.Union[int, float, list, object, tuple, None]:
        """
        read from shared variables

        :param var_name: variable name
        :type var_name: str

        :param index: index of agent (None or -1 if ALL_WRITE) which owns the variable
        :type index: int, None

        :return: value of variable
        :rtype: int, float, list, object, tuple, None

        """
        return self.__agent_gvh.get(var_name, index)

    # ------------ MUTEX INITIALIZATION, LOCKING, AND RELEASE METHODS --------------

    def initialize_lock(self, key: str) -> None:
        """
        create mutex

        :param key: name of lock (mutex)
        :type key: str
        """
        self.__mutexes[key] = mh_types.mutex_types[type(self.__mutex_handler)](key)
        self.__agent_gvh.mutex_handler.add_mutex(self.__mutexes[key])
        self.__mutexes[key].agent_comm_handler = self.__agent_comm_handler
        self.__requested_mutexes[key] = False
        self.__request_nums[key] = 0

    def lock(self, key: str) -> bool:
        """
        return False if doesn't have lock,  request lock if hasn't requested, return True if has lock.

        :param key: name of lock(mutex)
        :type key: str

        :return: whether or not this agent has the lock
        :rtype: bool
        """
        if not self.__requested_mutexes[key]:
            self.__mutexes[key].request_mutex(self.__request_nums[key])
            self.__requested_mutexes[key] = True
            self.__request_nums[key] += 1
            return False
        else:
            if not self.__agent_gvh.mutex_handler.has_mutex(self.__mutexes[key].mutex_id):
                return False
        return True

    def unlock(self, key: str) -> None:
        """
        unlock (release) mutex

        :param key: name of lock(mutex)
        :type key: str
        """
        self.__mutexes[key].release_mutex()
        self.__requested_mutexes[key] = False
        time.sleep(0.1)

    # ------------ EXIT METHODS --------------

    def stop(self) -> None:
        """
         a flag to set to to safely exit the thread
        """
        if self.__agent_gvh.moat is not None:
            self.__agent_gvh.moat.moat_exit_action()
            # todo:best termination message
            # TODO: send(msg,"",best_post,time.time()) best termination message.
        if self.__agent_gvh.mutex_handler is not None:
            if not self.__agent_gvh.mutex_handler.stopped():
                self.__agent_gvh.mutex_handler.stop()
        if self.__agent_comm_handler is not None:
            if not self.__agent_comm_handler.stopped():
                self.__agent_comm_handler.stop()
        if not self.stopped():
            self.__stop_event.set()
            print("stopping application thread on agent", self.pid())

    def stopped(self) -> bool:
        """
        set the stop flag

        :return: true if stop event is set, false otherwise
        :rtype: bool
        """
        return self.__stop_event.is_set()

    def signal_handler(self, sig, frame):
        """
        method for handling ctrl + C safely to stop agent thread.
        """
        self.stop()

    # ------------ APPLICATION AGNOSTIC METHODS --------------

    def msg_handle(self) -> None:
        """
        send unsent messages, handle received messages.
        """
        time.sleep(0.1)
        self.__agent_gvh.flush_msgs()
        self.__agent_comm_handler.handle_msgs()
        time.sleep(0.1)

    def set_idle(self) -> None:
        """
        send a message indicating this agent is ready to end the application
        """
        self.__do_loop_body = False
        stop_msg = mc.stop_msg_create(self.pid(), self.__agent_gvh.round_num, self.__agent_gvh.round_num)
        self.__agent_gvh.send(stop_msg)

    # ------------ ABSTRACT (APPLICATION SPECIFIC) METHODS --------------

    @abc.abstractmethod
    def initialize_vars(self) -> None:
        """
        abstract method to initialize variables
        """
        pass

    @abc.abstractmethod
    def loop_body(self) -> None:
        """
        loop body
        """
        pass

    # ------------ THREAD BEHAVIOR --------------

    def run(self) -> None:
        """
        run thread, behavior varies depending on initialize vars and loop body.
        """

        # synchronized initialization
        init_msg = mc.init_msg_create(self.pid(), self.__agent_gvh.round_num)

        while not self.__agent_gvh.init and not self.stopped():
            self.initialize_vars()
            self.msg_handle()
            self.__agent_gvh.send(init_msg)
            time.sleep(0.05)

        # update round number and execute loop body if agent hasn't requested to stop
        while not self.stopped():
            self.msg_handle()
            if not self.__agent_gvh.is_alive:
                self.__do_loop_body = False
                self.stop()
                continue

            # synchronized round based loop behavior
            try:
                round_update_msg = mc.round_update_msg_create(self.pid(), self.__agent_gvh.round_num,
                                                              self.__agent_gvh.round_num)
                while not self.__agent_gvh.update_round:
                    if self.stopped():
                        break
                    self.msg_handle()
                    self.__agent_gvh.send(round_update_msg)
                    time.sleep(0.1)

                if self.stopped():
                    break

                self.msg_handle()
                if self.__do_loop_body:
                    self.loop_body()
                self.__agent_gvh.update_round = False

            except OSError:
                print("some unhandled error in application thread for agent", self.pid(), sys.exc_info())
                self.set_idle()

            if self.__agent_comm_handler.stopped():
                self.set_idle()
