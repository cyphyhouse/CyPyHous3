import time
from typing import Union

from src.config.configs import AgentConfig
from src.functionality.comm_funcs import send
from src.functionality.synchronizer import Synchronizer
from src.harness.comm_handler import CommHandler
from src.harness.message_handler import stop_msg_confirm_create, init_msg_confirm_create, dsm_update_create, \
    round_update_msg_confirm_create
from src.objects.dsm import DSM
from src.objects.message import Message


class Gvh(object):
    """
    global variable holder for the agent, this contains the worldview of each agent
    handles distributed shared memory.

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

    def __init__(self, a: AgentConfig):
        """
        init method for global variable holder object
        :param a: Agent configuration
        """
        self.__pid = a.pid
        self.__participants = a.bots
        self.__msg_list = []
        self.__port_list = a.plist
        self.__rport = a.rport
        self.__is_leader = a.is_leader
        self.__is_alive = True
        self.__comm_handler = CommHandler(a)
        self.__dsm = []
        self.__synchronizer = None
        self.__mutex_handler = a.mutex_handler
        self.__window = 1

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

    def start_mh(self):
        if self.mutex_handler is not None:
            self.mutex_handler.start()

    @property
    def comm_handler(self) -> CommHandler:
        """
        getter method for the communication handler
        :return:
        """
        return self.__comm_handler

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
        self._broadcast(msg)

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
            self._broadcast(msg)

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

    def flush_msgs(self) -> None:
        """
        Send all messages to be sent; then process all received messages in queue
        """
        # Send messages
        for msg in self.__msg_list:
            self._broadcast(msg)
        self.__msg_list = []

        # Process received messages
        curr_size = self.__comm_handler.recv_msg_queue.qsize()
        # Since only AgentThread is calling this function, other threads will only increase qsize
        for _ in range(0, curr_size):
            received_msg = self.__comm_handler.recv_msg_queue.get()
            if received_msg.message_type in message_handler:
                message_handler[received_msg.message_type](received_msg, self)
            else:
                print("Warning: unexpected message type id", received_msg.message_type)

    def _broadcast(self, msg: Message) -> None:
        for i in range(self.__window):
            send(msg, AgentConfig.BROADCAST_ADDR, self.__rport, self.__window)


def round_update_msg_handle(msg: Message, agent_gvh: Gvh):
    rounding,round_num = msg.sender,msg.content
    if agent_gvh.is_leader:
        if rounding in agent_gvh.round_counter:
            pass
        else:
            #print(rounding, agent_gvh.round_counter)
            agent_gvh.round_counter.append(rounding)
        leaderid = -1
        if agent_gvh.is_leader:
            leaderid = agent_gvh.pid

        msg_contents = (leaderid, agent_gvh.round_num)
        msg1 = round_update_msg_confirm_create(agent_gvh.pid, msg_contents, agent_gvh.round_num)
        if len(agent_gvh.round_counter) == agent_gvh.participants:
            #print("sending message to update round", agent_gvh.round_num)
            agent_gvh._broadcast(msg1)


def round_update_msg_confirm_handle(msg: Message, agent_gvh: Gvh):
    leaderid , roundnum = int(msg.content[0]), int(msg.content[1])
    #print("got message to update round", roundnum)
    if roundnum < agent_gvh.round_num:
        pass
    elif msg.sender == leaderid:
        agent_gvh.update_round = True
        agent_gvh.round_num = roundnum+1
        agent_gvh.start_time = time.time()
        agent_gvh.round_counter = []


def stop_msg_handle(msg: Message, agent_gvh: Gvh):
    stopping = msg.sender
    if agent_gvh.is_leader:

        if stopping in agent_gvh.stop_counter :
            pass
        else:
            agent_gvh.stop_counter.append(stopping)
        leaderid = -1
        if agent_gvh.is_leader:
            leaderid = agent_gvh.pid

        msg1 = stop_msg_confirm_create(agent_gvh.pid, leaderid, agent_gvh.round_num)
        if len(agent_gvh.stop_counter) == agent_gvh.participants:
            agent_gvh._broadcast(msg1)


def stop_msg_confirm_handle(msg: Message, agent_gvh: Gvh):
    leaderid = int(msg.content)
    if int(msg.sender) == leaderid:
        agent_gvh.is_alive = False


def init_msg_handle(msg: Message, agent_gvh: Gvh):
    initing = msg.sender
    if agent_gvh.is_leader:
        if initing in agent_gvh.init_counter:
            pass
        else:
            agent_gvh.init_counter.append(initing)
        leaderid = -1
        if agent_gvh.is_leader:
            leaderid = agent_gvh.pid
        msg1 = init_msg_confirm_create(agent_gvh.pid, leaderid, time.time())
        if len(agent_gvh.init_counter) == agent_gvh.participants:
            agent_gvh._broadcast(msg1)


def init_msg_confirm_handle(msg: Message, agent_gvh: Gvh):
    if msg.sender == msg.content:
        agent_gvh.init = True
        agent_gvh.start_time = time.time()


def base_mutex_ack_handle(msg: Message, agent_gvh: Gvh) -> None:
    mutex_name, reqnum, grantee = msg.content
    if grantee == agent_gvh.pid:
        agent_gvh.ack_nums[mutex_name] = int(reqnum)

    pass


def round_update_handle(msg: Message, agent_gvh: Gvh) -> None:
    """
    update number of agents reaching barrier
    :param msg:
    :param agent_gvh: agent gvh handling updates
    :return:
    """
    try:
        agent_gvh.synchronizer.handle_sync_message(msg)
    except:
        print("error")


def base_mutex_request_handle(msg: Message, agent_gvh: Gvh) -> None:
    """
    add request to list of requests
    :param msg: request message
    :param agent_gvh: my gvh
    :return: nothing
    """
    mutex_id, req_num = msg.content
    requester = msg.sender
    if agent_gvh.is_leader:
        agent_gvh.mutex_handler.add_request(mutex_id, requester, req_num)
    else:
        pass


def base_mutex_grant_handle(msg: Message, agent_gvh: Gvh) -> None:
    """
    grant first request
    :param msg: grant message
    :param agent_gvh: my gvh
    :return: nothing
    """
    mutex_id, grantee, mutexnum = msg.content
    index = agent_gvh.mutex_handler.find_mutex_index(mutex_id)
    agent_gvh.mutex_handler.mutexes[index].mutex_holder = grantee


def base_mutex_release_handle(msg: Message, agent_gvh: Gvh) -> None:
    """
    release mutex held
    :param msg: release message
    :param agent_gvh: my gvh
    :return: nothing
    """
    mutex_id = msg.content
    releaser = msg.sender
    i = agent_gvh.mutex_handler.find_mutex_index(mutex_id)
    if agent_gvh.is_leader:
        if agent_gvh.mutex_handler.mutexes[i].mutex_holder == releaser:
            agent_gvh.mutex_handler.mutexes[i].mutex_holder = None
            agent_gvh.mutex_handler.mutexes[i].mutex_request_list = agent_gvh.mutex_handler.mutexes[i].mutex_request_list[1:]
    else:
        pass


def stop_comm_msg_handle(msg: Message, agent_gvh: Gvh) -> None:
    """
    stop comm handler on receiving this message
    :param msg:
    :param agent_gvh:
    :return:
    """
    if msg.sender == agent_gvh.comm_handler.pid:
        print("stopping comm_handler on agent", agent_gvh.comm_handler.pid)
        agent_gvh.comm_handler.stop()


def message_update_handle(msg: Message, agent_gvh: Gvh):
    var = msg.content
    updater = msg.sender
    for i in range(len(agent_gvh.dsm)):

        if agent_gvh.dsm[i].name == var.name:
            if var.owner == 0:

                if agent_gvh.dsm[i].updated is not None and agent_gvh.dsm[i].updated > msg.timestamp:
                    pass
                else:
                    agent_gvh.dsm[i] = var
                    agent_gvh.dsm[i].updated = msg.timestamp

            else:

                if agent_gvh.dsm[i].get_val(updater) is None or agent_gvh.dsm[i].last_update(updater) is None:

                    agent_gvh.dsm[i].set_val(var.get_val(updater), updater)
                    agent_gvh.dsm[i].set_update(msg.timestamp, updater)

                elif (agent_gvh.dsm[i].last_update(updater)) > (msg.timestamp):
                    pass
                    #print("last update from", updater, "at ", agent_gvh.dsm[i].last_update(updater)," overriding", msg.timestamp)
                else:

                    agent_gvh.dsm[i].set_val(var.get_val(updater), updater)
                    agent_gvh.dsm[i].set_update(int(msg.timestamp), updater)


message_handler = dict()

message_handler[0] = round_update_handle
message_handler[1] = base_mutex_request_handle
message_handler[2] = base_mutex_grant_handle
message_handler[3] = base_mutex_release_handle
message_handler[4] = message_update_handle
message_handler[5] = stop_comm_msg_handle
message_handler[6] = base_mutex_ack_handle
message_handler[7] = init_msg_handle
message_handler[8] = init_msg_confirm_handle
message_handler[9] = round_update_msg_handle
message_handler[10] = round_update_msg_confirm_handle
message_handler[11] = stop_msg_handle
message_handler[12] = stop_msg_confirm_handle
