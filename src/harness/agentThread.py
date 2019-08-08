import signal
import time
from abc import ABC, abstractmethod
from threading import Thread, Event
from typing import Union

from src.config.configs import AgentConfig, MoatConfig
from src.functionality.comm_funcs import send
from src.harness.comm_handler import CommHandler
from src.harness.gvh import Gvh
from src.harness.message_handler import stop_comm_msg_create
from src.objects.base_mutex import BaseMutex


class AgentThread(ABC, Thread):
    """
    abstract object for each agent thread/application. This contains
    the methods and fields each agent application must implement.
    """

    def __init__(self, agent_config: AgentConfig, moat_config: Union[MoatConfig, None]) -> None:
        """

        :param agent_config:
        :param moat_config:
        """
        super(AgentThread, self).__init__()
        self.__agent_gvh = Gvh(agent_config, moat_config)
        self.__agent_comm_handler = CommHandler(agent_config, self.__agent_gvh)
        self.__agent_gvh.start_moat()
        self.__agent_gvh.start_mh()
        self.__stop_event = Event()
        self.__mutex_handler = self.__agent_gvh.mutex_handler
        self.requestedlocks = {}
        self.req_nums = {}
        self.baselocks = {}
        self.locals = {}

        # create a signal handler to handle ctrl + c
        signal.signal(signal.SIGINT, self.signal_handler)
        # create init messages, and keep sending until leader acks, then start app thread.
        # leader only starts once everyone has received an ack.

    def create_ar_var(self, name, dtype, initial_value=None):
        self.agent_gvh.create_ar_var(name, dtype, initial_value)
        pass

    def create_aw_var(self, name, dtype, initial_value=None):
        self.agent_gvh.create_aw_var(name, dtype, initial_value)
        pass

    def initialize_lock(self, key):
        self.baselocks[key] = BaseMutex(1, self.agent_gvh.port_list)
        self.agent_gvh.mutex_handler.add_mutex(self.baselocks[key])
        self.baselocks[key].agent_comm_handler = self.agent_comm_handler
        self.requestedlocks[key] = False
        self.req_nums[key] = 0

    def lock(self, key: str):
        # print("checking locking")
        if not self.requestedlocks[key]:
            self.baselocks[key].request_mutex(self.req_nums[key])
            self.requestedlocks[key] = True
            self.req_nums[key] += 1
            # print("requesting")
            return False
        else:
            if not self.agent_gvh.mutex_handler.has_mutex(self.baselocks[key].mutex_id):
                # print("requested not granted")
                return False
        return True

    def unlock(self, key: str):
        self.baselocks[key].release_mutex()
        self.requestedlocks[key] = False
        time.sleep(0.1)

    @property
    def locals(self):
        return self.__locals

    @locals.setter
    def locals(self, locals):
        self.__locals = locals

    @property
    def agent_gvh(self) -> Gvh:
        """
        getter method for agent gvh
        :return: gvh of the current agent thread object
        """
        return self.__agent_gvh

    @agent_gvh.setter
    def agent_gvh(self, agent_gvh: Gvh) -> None:
        """
        setter method for gvh for the current agent thread object
        :param agent_gvh: new Gvh object to be set
        :return: None
        """
        self.__agent_gvh = agent_gvh

    @property
    def agent_comm_handler(self) -> CommHandler:
        """
        getter method for agent comm handler
        :return: the communication handler of the agent thread object
        """
        return self.__agent_comm_handler

    @agent_comm_handler.setter
    def agent_comm_handler(self, agent_comm_handler) -> None:
        """
        setter method for agent comm handler
        :param agent_comm_handler: comm handler object to be set
        :return: None
        """
        self.__agent_comm_handler = agent_comm_handler

    def stop(self) -> None:
        """
         a flag to set to to safely exit the thread
        :return: None
        """
        if self.agent_gvh.moat is not None:
            self.agent_gvh.moat.moat_exit_action()
            # todo: msg = tERMINATE_MSG() best termination message
            # TODO: send(msg,"",best_post,time.time()) best termination message.
        if self.agent_gvh.mutex_handler is not None:
            if not self.agent_gvh.mutex_handler.stopped():
                self.agent_gvh.mutex_handler.stop()
        if self.agent_comm_handler is not None:
            send(stop_comm_msg_create(self.pid(), time.time()), "<broadcast>", self.receiver_port())
            # signal.pthread_kill(self.agent_comm_handler.ident, signal.SIGINT)
            if not self.agent_comm_handler.stopped():
                self.agent_comm_handler.stop()
        if not self.stopped():
            self.__stop_event.set()
            print("stopped application thread on agent", self.pid())

    def stopped(self) -> bool:
        """
        set the stop flag
        :return: true if stop event is set, false otherwise
        """
        return self.__stop_event.is_set()

    def pid(self) -> int:
        """
        get the pid of the current agent.
        uses the agent gvh to retrieve it
        :return: integer pid of the current agent
        """
        return self.agent_gvh.pid

    def num_agents(self) -> int:
        """
        method to return the number of participants in the system
        :return: integer number of participants
        """
        return self.agent_gvh.participants

    def receiver_port(self) -> int:
        """
        gets the port the application is receiving messages on
        :return: integer receiver port
        """
        return self.agent_comm_handler.r_port

    def receiver_ip(self) -> str:
        """
        method to get the ip of receiver
        :return: string ip of receiver
        """
        return self.agent_comm_handler.ip

    def signal_handler(self, sig, frame):
        """
        method for handling ctrl + C safely to stop agent thread.
        :param sig:
        :param frame:
        :return:
        """
        self.stop()

    def msg_handle(self):
        time.sleep(0.1)
        self.agent_gvh.flush_msgs()
        self.agent_comm_handler.handle_msgs()
        time.sleep(0.1)

    @abstractmethod
    def initialize_vars(self):
        """
        abstract method to initialize variables
        :return:
        """
        pass

    @abstractmethod
    def loop_body(self):
        """
        loop body
        :return:
        """
        pass

    def write_to_shared(self, var_name, index, value):
        if index is not None:
            self.agent_gvh.put(var_name, value, index)
        else:
            self.agent_gvh.put(var_name, value)

    def read_from_shared(self, var_name, index):
        if index is not None:
            return self.agent_gvh.get(var_name, index)
        else:
            return self.agent_gvh.get(var_name)
        pass

    def run(self) -> None:
        """
        needs to be implemented for any agenThread
        :return:
        """
        self.initialize_vars()

        while not self.stopped():

            self.msg_handle()
            try:
                self.loop_body()
            except OSError:
                print("some unhandled error in application thread for agent", self.pid())
                self.stop()

            if self.agent_comm_handler.stopped():
                self.stop()
