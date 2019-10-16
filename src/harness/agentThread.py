import time
from abc import ABC, abstractmethod
from threading import Thread, Event
from typing import Union

from src.config.configs import AgentConfig, MoatConfig
from src.functionality.comm_funcs import send
from src.harness.comm_handler import CommHandler
from src.harness.gvh import Gvh
from src.harness.message_handler import stop_comm_msg_create, round_update_msg_create, stop_msg_create
from src.motion.moat_withlidar import MoatWithLidar
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
        self.ackedlocks = {}

        self.baselocks = {}
        self.locals = {}
        self.initialize_vars()

        # create init messages, and keep sending until leader acks, then start app thread.
        # leader only starts once everyone has received an ack.

    def create_ar_var(self, name, dtype, initial_value=None):
        self.agent_gvh.create_ar_var(name, dtype, initial_value)
        pass

    def create_aw_var(self, name, dtype, initial_value=None):
        self.agent_gvh.create_aw_var(name, dtype, initial_value)
        pass

    def initialize_lock(self, key):
        self.baselocks[key] = BaseMutex(key, self.agent_gvh.port_list)
        self.agent_gvh.mutex_handler.add_mutex(self.baselocks[key])
        self.baselocks[key].agent_comm_handler = self.agent_comm_handler
        self.requestedlocks[key] = False
        self.ackedlocks[key] = False
        self.agent_gvh.req_nums[key] = 0
        self.agent_gvh.ack_nums[key] = -1

    def lock(self, key: str):
        if self.agent_gvh.ack_nums[key] == self.agent_gvh.req_nums[key]:
            self.ackedlocks[key] = True

        # print("checking locking")
        if not self.requestedlocks[key]:
            self.baselocks[key].request_mutex(self.agent_gvh.req_nums[key])
            self.requestedlocks[key] = True
            return False
        elif not self.ackedlocks[key]:
            if not self.agent_gvh.mutex_handler.has_mutex(self.baselocks[key].mutex_id):
                self.baselocks[key].request_mutex(self.agent_gvh.req_nums[key])
                return False
        else:
            if not self.agent_gvh.mutex_handler.has_mutex(self.baselocks[key].mutex_id):
                return False

        self.agent_gvh.req_nums[key] += 1
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
            send(stop_comm_msg_create(self.pid(), time.time()), AgentConfig.BROADCAST_ADDR, self.receiver_port())
            if not self.agent_comm_handler.stopped():
                self.agent_comm_handler.stop()
                self.agent_comm_handler.join()  # Wait until comm_handler finishes
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

    def release_unnecessary_mutexes(self):
        for i in list(self.baselocks.keys()):
            if self.agent_gvh.mutex_handler.has_mutex(self.baselocks[i].mutex_id):
                if not self.requestedlocks[i]:
                    self.baselocks[i].release_mutex()

    def trystop(self):
        stop_msg = stop_msg_create(self.pid(), self.agent_gvh.round_num, self.agent_gvh.round_num)
        if len(self.agent_gvh.port_list) is not 0:
            for port in self.agent_gvh.port_list:
                send(stop_msg, AgentConfig.BROADCAST_ADDR, port)
        else:
            send(stop_msg, AgentConfig.BROADCAST_ADDR, self.receiver_port())

    def run(self) -> None:
        """
        needs to be implemented for any agenThread
        :return:
        """
        from src.harness.message_handler import init_msg_create
        init_msg = init_msg_create(self.pid(), self.agent_gvh.round_num)
        while not self.agent_gvh.init:
            # print("sending init", self.pid())
            self.initialize_vars()
            self.msg_handle()
            if len(self.agent_gvh.port_list) is not 0:
                for port in self.agent_gvh.port_list:
                    send(init_msg, AgentConfig.BROADCAST_ADDR, port)
            else:
                send(init_msg, AgentConfig.BROADCAST_ADDR, self.receiver_port())
            time.sleep(0.05)

        while not self.stopped():
            self.msg_handle()
            self.release_unnecessary_mutexes()
            if not self.agent_gvh.is_alive:
                print("stopping app thread on ", self.pid())
                self.stop()
                continue

            try:
                round_update_msg = round_update_msg_create(self.pid(), self.agent_gvh.round_num,
                                                           self.agent_gvh.round_num)
                while not self.agent_gvh.update_round:
                    if self.stopped():
                        break

                    # print("sending init", self.pid())
                    self.msg_handle()
                    if len(self.agent_gvh.port_list) is not 0:
                        for port in self.agent_gvh.port_list:
                            send(round_update_msg, AgentConfig.BROADCAST_ADDR, port)
                    else:
                        send(round_update_msg, AgentConfig.BROADCAST_ADDR, self.receiver_port())
                    time.sleep(0.1)

                if self.stopped():
                    break
                # print("executing round", self.agent_gvh.round_num)

                self.loop_body()
                # resetting t_pos and t_sync
                if self.agent_gvh.moat is not None:
                    if isinstance(self.agent_gvh.moat, MoatWithLidar):
                        self.agent_gvh.moat.tscan = []
                        self.agent_gvh.moat.tpos = {}

                self.agent_gvh.update_round = False

            except OSError as e:
                print("some unhandled error in application thread for agent", e)
                self.trystop()

            if self.agent_comm_handler.stopped():
                self.trystop()
