# functions for message handling. use dictionary to call them
import time
from typing import Any, Type, Union

import message
from gvh import Gvh


def update_compound_create(data_type: tuple, pid: int, var: str, value: Any, ts: float) -> message.Message:
    """
    TODO:create message to update compound variable type
    """
    pass


def round_update_create(pid:int, ts: float) -> message.Message:
    """
    creates an end of round message to let other agents know i have reached my end
    :param ts:
    :return:
    """
    return message.Message(6, pid, "", ts)


def round_update_handle(msg: message.Message, agent_gvh: Gvh) -> None:
    """
    update number of agents reaching barrier
    :param msg:
    :param agent_gvh:
    :return:
    """
    if msg.sender not in agent_gvh.finished:
        agent_gvh.finished.append(msg.sender)
    if len(agent_gvh.finished) == agent_gvh.participants - 1:
        agent_gvh.start_round = True
        agent_gvh.finished = []


def mutex_request_create(var_name: str, pid: int, ts: float) -> message.Message:
    """
    create mutex request
    :param var_name: variable to create mutex request for
    :param pid: pid of agent requesting mutex
    :param ts: time stamp
    :return: mutex request message
    """
    message_str = var_name
    return message.Message(3, pid, message_str, ts)


def mutex_grant_create(var_name: str, grantee: int, pid: int, ts: float) -> message.Message:
    """
    grant mutex message creation
    :param var_name: variable to grant mutex on
    :param grantee: pid of agent that agent is granting mutex to
    :param pid: leader pid who is granting the mutex
    :param ts: timestamp
    :return: mutex grant message
    """
    message_str = var_name + "," + str(grantee)
    return message.Message(4, pid, message_str, ts)


def mutex_release_create(var_name: str, pid: int, ts: float) -> message.Message:
    """
    release held mutex
    :param var_name: variable name
    :param pid: releasing agent's pid
    :param ts: timestamp
    :return: mutex release
    """
    message_str = var_name
    return message.Message(5, pid, message_str, ts)


def mutex_request_handle(msg: message.Message, agent_gvh: Gvh) -> None:
    """
    add request to list of requests
    :param msg: request message
    :param agent_gvh: my gvh
    :return: nothing
    """
    var_name = msg.content
    requester = msg.sender
    i = agent_gvh.get_mutex_index(var_name)
    holder = agent_gvh.mutex_list[i].mutex_holder
    requests = agent_gvh.mutex_list[i].requests
    if agent_gvh.is_leader:
        if requests == [] and holder is None:
            agent_gvh.mutex_list[i].mutex_holder = requester
            msg = mutex_grant_create(var_name, requester, agent_gvh.pid, time.time())
            agent_gvh.add_msg(msg)
        else:
            agent_gvh.mutex_list[i].requests.append(requester)

    else:
        pass


def mutex_grant_handle(msg: message.Message, agent_gvh: Gvh) -> None:
    """
    grant first request
    :param msg: grant message
    :param agent_gvh: my gvh
    :return: nothing
    """
    var_name, grantee = msg.content.split(",")
    index = agent_gvh.get_mutex_index(var_name)
    agent_gvh.mutex_list[index].mutex_holder = agent_gvh.pid
    pass


def mutex_release_handle(msg: message.Message, agent_gvh: Gvh) -> None:
    """
    release mutex held
    :param msg: release message
    :param agent_gvh: my gvh
    :return: nothing
    """
    varname = msg.content
    i = agent_gvh.get_mutex_index(varname)
    if agent_gvh.is_leader:
        if agent_gvh.mutex_list[i].mutex_holder == msg.sender:
            if not agent_gvh.mutex_list[i].requests == []:
                holder = agent_gvh.mutex_list[i].requests[0]
                agent_gvh.mutex_list[i].mutex_holder = holder
                agent_gvh.mutex_list[i].requests = agent_gvh.mutex_list[i].requests[1:]
                msg = mutex_grant_create(varname, agent_gvh.pid, holder, time.time())
                agent_gvh.add_msg(msg)
            else:
                pass
    else:
        pass


def update_create(data_type: Type[Union[int, bool, str, float]],
                  pid: int, var: str, value: Any, ts: float) -> message.Message:
    """
    create message to update variable
    :param data_type: data type of variable to cast
    :param pid: pid of sender
    :param var: variable name
    :param value: value to update to
    :param ts: timestamp
    :return: message
    """
    message_str = var + "," + str(value)
    msg = message.Message(update_message_type[data_type], pid, message_str, ts)
    return msg


def update_handle(msg: message.Message, agent_gvh: Gvh) -> None:
    """
    :param msg: handle update message
    :param agent_gvh: my gvh
    :return: nothing
    """
    if 0 < msg.m_type <= 2:
        var_name, value = msg.content.split(",")
        agent_gvh.agent_dsm.put(msg.sender, var_name, update_type[msg.m_type](value))


update_message_type = dict()
update_message_type[int] = 1
update_message_type[bool] = 2

update_type = dict()
update_type[1] = int
update_type[2] = bool

message_handler = dict()
message_handler[1] = update_handle
message_handler[2] = update_handle
message_handler[3] = mutex_request_handle
message_handler[4] = mutex_grant_handle
message_handler[5] = mutex_release_handle
message_handler[6] = round_update_handle
