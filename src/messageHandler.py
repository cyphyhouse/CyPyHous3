from typing import Any, Type, Union
import time

import message
from gvh import Gvh


def update_compound_create(data_type: tuple, pid: int, var: str, value: Any, ts: float) -> message.Message:
    """
    create message to update compound variable type
    """
    pass


def mutex_request_create(var_name: str, pid: int, ts: float) -> message.Message:
    """
    create mutex request
    :param var_name:
    :param pid:
    :param ts:
    :return:
    """
    message_str = var_name
    return message.Message(3, pid, message_str, ts)


def mutex_grant_create(var_name: str, grantee: int, pid: int, ts: float) -> message.Message:
    """
    grant mutex
    :param var_name:
    :param grantee:
    :param pid:
    :param ts:
    :return:
    """
    message_str = var_name + "," + str(grantee)
    return message.Message(4, pid, message_str, ts)


def mutex_release_create(var_name: str, pid: int, ts: float) -> message.Message:
    """
    release held mutex
    :param var_name:
    :param pid:
    :param ts:
    :return:
    """
    message_str = var_name
    return message.Message(5, pid, message_str, ts)


def mutex_request_handle(msg: message.Message, agent_gvh: Gvh) -> None:
    """
    add request to list of requests
    :param msg:
    :param agent_gvh:
    :return:
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
    :param msg:
    :param agent_gvh:
    :return:
    """
    var_name, grantee = msg.content.split(",")
    index = agent_gvh.get_mutex_index(var_name)
    agent_gvh.mutex_list[index].mutex_holder = agent_gvh.pid
    print("granting mutex to", agent_gvh.pid)
    pass


def mutex_release_handle(msg: message.Message, agent_gvh: Gvh) -> None:
    """
    release mutex held
    :param msg:
    :param agent_gvh:
    :return:
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
    :param data_type:
    :param pid:
    :param var:
    :param value:
    :param ts:
    :return: message
    """
    message_str = var + "," + str(value)
    msg = message.Message(update_message_type[data_type], pid, message_str, ts)
    return msg


def update_handle(msg: message.Message, agent_gvh: Gvh) -> None:
    """
    :param msg:
    :param agent_gvh:
    :return:
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
