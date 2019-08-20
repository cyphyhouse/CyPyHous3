# functions for message handling. use dictionary to call them

import time

import src.objects.message as message
from src.functionality.comm_funcs import send
from src.harness.gvh import Gvh


def init_msg_handle(msg: message.Message, agent_gvh: Gvh):
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
            if len(agent_gvh.port_list) is not 0:
                for port in agent_gvh.port_list:
                    send(msg1, "<broadcast>", port)
            else:
                send(msg1, "<broadcast>", agent_gvh.rport)


def init_msg_confirm_handle(msg: message.Message, agent_gvh: Gvh):
    if msg.sender == msg.content:
        agent_gvh.init = True


def init_msg_create(pid, ts: float):
    return message.Message(pid, 7, None, ts)


def init_msg_confirm_create(pid, leaderid, ts: float):
    return message.Message(pid, 8, leaderid, ts)


def base_mutex_ack_create(pid, grantee, mutex_name, reqnum, ts: float) -> message.Message:
    msg_contents = (mutex_name, reqnum, grantee)
    return message.Message(pid, 6, msg_contents, ts)


def base_mutex_ack_handle(msg: message.Message, agent_gvh: Gvh) -> None:
    mutex_name, reqnum, grantee = msg.content
    if grantee == agent_gvh.pid:
        agent_gvh.ack_nums[mutex_name] = int(reqnum)

    pass


def stop_comm_msg_create(pid: int, ts: float) -> message.Message:
    """
    create a message to stop the comm_handler
    :param pid:
    :param ts:
    :return:
    """
    return message.Message(pid, 5, None, ts)


def round_update_create(pid: int, round_num: int, ts: float) -> message.Message:
    """
    creates an end of round message to let other agents know i have reached my end
    :param pid : pid of sender
    :param round_num : round number being synced.
    :param ts: timestamp
    :return:
    """
    return message.Message(pid, 0, round_num, ts)


def base_mutex_request_create(mutex_id: int, req_num: int, pid: int, ts: float) -> message.Message:
    """
    create mutex request
    :param mutex_id: variable to create mutex request for
    :param pid: pid of agent requesting mutex
    :param req_num : request number
    :param ts: time stamp
    :return: mutex request message
    """
    message_contents = (mutex_id, req_num)
    return message.Message(pid, 1, message_contents, ts)


def base_mutex_grant_create(mutex_id: int, agent_id: int, pid: int, mutexnum: int, ts: float) -> message.Message:
    """
    grant mutex message creation
    :param mutexnum :mutex number
    :param mutex_id: variable to grant mutex on
    :param agent_id: agent to grant mutex to
    :param pid: leader pid who is granting the mutex
    :param ts: timestamp
    :return: mutex grant message
    """
    message_contents = (mutex_id, agent_id, mutexnum)
    return message.Message(pid, 2, message_contents, ts)


def mutex_release_create(mutex_id: int, pid: int, ts: float) -> message.Message:
    """
    release held mutex
    :param mutex_id: variable name
    :param pid: releasing agent's pid
    :param ts: timestamp
    :return: mutex release
    """
    message_contents = mutex_id
    return message.Message(pid, 3, message_contents, ts)


def round_update_handle(msg: message.Message, agent_gvh: Gvh) -> None:
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

def test_mesg_create(seqnum:int, pid:int):
    return message.Message(pid,9,seqnum,time.time())

def test_mesg_update_handle(msg: message.Message, agent_gvh: Gvh) -> None:
    """
    update number of agents reaching barrier
    :param msg:
    :param agent_gvh: agent gvh handling updates
    :return:
    """
    print("received message id", int(msg.content), "from ",int(msg.sender))


def base_mutex_request_handle(msg: message.Message, agent_gvh: Gvh) -> None:
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


def base_mutex_grant_handle(msg: message.Message, agent_gvh: Gvh) -> None:
    """
    grant first request
    :param msg: grant message
    :param agent_gvh: my gvh
    :return: nothing
    """
    mutex_id, grantee, mutexnum = msg.content
    index = agent_gvh.mutex_handler.find_mutex_index(mutex_id)
    agent_gvh.mutex_handler.mutexes[index].mutex_holder = grantee


def base_mutex_release_handle(msg: message.Message, agent_gvh: Gvh) -> None:
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


def stop_comm_msg_handle(msg: message.Message, agent_gvh: Gvh) -> None:
    """
    stop comm handler on receiving this message
    :param msg:
    :param agent_gvh:
    :return:
    """
    pass


def message_update_handle(msg: message.Message, agent_gvh: Gvh):
    # print("got an update message",msg)
    var = msg.content
    updater = msg.sender
    for i in range(len(agent_gvh.dsm)):

        if agent_gvh.dsm[i].name == var.name:
            if var.owner == 0:

                if agent_gvh.dsm[i].updated is not None and agent_gvh.dsm[i].updated > float(msg.timestamp):
                    pass
                else:
                    agent_gvh.dsm[i] = var
                    agent_gvh.dsm[i].updated = float(msg.timestamp)

            else:
                if agent_gvh.dsm[i].get_val(updater) is None or agent_gvh.dsm[i].last_update(updater) is None:
                    agent_gvh.dsm[i].set_val(var.get_val(updater), updater)
                    agent_gvh.dsm[i].set_update(float(msg.timestamp), updater)

                elif agent_gvh.dsm[i].last_update(updater) > float(msg.timestamp):
                    pass
                else:
                    agent_gvh.dsm[i].set_val(var.get_val(updater), updater)
                    agent_gvh.dsm[i].set_update(float(msg.timestamp), updater)


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
message_handler[9] = test_mesg_update_handle
