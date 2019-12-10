# Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

import time

import src.objects.message as message
from src.datatypes.message_types import MsgType
from src.harness.gvh import Gvh
from src.harness.msg_create import *


def round_update_msg_handle(msg: message.Message, agent_gvh: Gvh):
    """
    message handling for round update message
    :param msg: message to be handled
    :param agent_gvh: agent gvh
    """
    rounding, round_num = msg.sender, msg.content
    if agent_gvh.is_leader:
        if rounding in agent_gvh.round_counter:
            pass
        else:
            agent_gvh.round_counter.append(rounding)
        leaderid = -1
        if agent_gvh.is_leader:
            leaderid = agent_gvh.pid

        msg1 = round_update_msg_confirm_create(agent_gvh.pid, leaderid, agent_gvh.round_num, time.time())
        if len(agent_gvh.round_counter) == agent_gvh.nsys:
            agent_gvh.send(msg1)


def round_update_msg_confirm_handle(msg: message.Message, agent_gvh: Gvh):
    """
    handle round update confirm message
    :param msg: message to be handled
    :param agent_gvh: agent gvh
    """
    leaderid, roundnum = int(msg.content[0]), int(msg.content[1])
    # print("got message to update round", roundnum)
    if roundnum < agent_gvh.round_num:
        pass
    elif msg.sender == leaderid:
        agent_gvh.update_round = True
        agent_gvh.round_num = roundnum + 1
        agent_gvh.start_time = time.time()
        agent_gvh.round_counter = []


def stop_msg_handle(msg: message.Message, agent_gvh: Gvh):
    """
    message handler for stop message
    :param msg: stop message
    :param agent_gvh: agent gvh
    """
    stopping = msg.sender
    if agent_gvh.is_leader:

        if stopping in agent_gvh.stop_counter:
            pass
        else:
            agent_gvh.stop_counter.append(stopping)
        leaderid = -1
        if agent_gvh.is_leader:
            leaderid = agent_gvh.pid

        msg1 = stop_msg_confirm_create(agent_gvh.pid, leaderid, agent_gvh.round_num)
        if len(agent_gvh.stop_counter) == agent_gvh.nsys:
            agent_gvh.send(msg1)


def stop_msg_confirm_handle(msg: message.Message, agent_gvh: Gvh):
    """
    confirm stop message confirmation
    :param msg: message to be handled
    :param agent_gvh: agent gvh
    """
    leaderid = int(msg.content)
    if int(msg.sender) == leaderid:
        agent_gvh.is_alive = False


def init_msg_handle(msg: message.Message, agent_gvh: Gvh):
    """
    init message handler
    :param msg: msg to be handled
    :param agent_gvh: agent gvh
    """
    initing = msg.sender
    if agent_gvh.is_leader:
        if initing in agent_gvh.init_counter:
            pass
        else:
            agent_gvh.init_counter.append(initing)
        leaderid = -1
        if agent_gvh.is_leader:
            leaderid = agent_gvh.pid
        msg1 = init_msg_confirm_create(agent_gvh.pid, leaderid, agent_gvh.round_num)
        if len(agent_gvh.init_counter) == agent_gvh.nsys:
            agent_gvh.send(msg1)


def init_msg_confirm_handle(msg: message.Message, agent_gvh: Gvh):
    """
    handle init confirmation message
    :param msg: message to be handled
    :param agent_gvh: agent gvh
    """
    if msg.sender == msg.content:
        agent_gvh.init = True
        agent_gvh.start_time = time.time()


def base_mutex_request_handle(msg: message.Message, agent_gvh: Gvh) -> None:
    """
    add request to list of requests
    :param msg: request message
    :param agent_gvh: my gvh
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
    """
    mutex_id, grantee, mutexnum = msg.content
    try:
        agent_gvh.mutex_handler.mutexes[mutex_id].mutex_holder = grantee
    except KeyError:
        print("tried granting possibly undeclared mutex")


def base_mutex_release_handle(msg: message.Message, agent_gvh: Gvh) -> None:
    """
    release mutex held
    :param msg: release message
    :param agent_gvh: my gvh
    :return: nothing
    """
    mutex_id = msg.content
    releaser = msg.sender
    if agent_gvh.is_leader:
        try:
            if agent_gvh.mutex_handler.mutexes[mutex_id].mutex_holder == releaser:
                agent_gvh.mutex_handler.mutexes[mutex_id].mutex_holder = None

        except KeyError:
            print("tried releasing possibly undeclared mutex")
    else:
        pass


def stop_comm_msg_handle(msg: message.Message, agent_gvh: Gvh) -> None:
    """
    dummy message to stop listening port.
    """
    pass


def message_update_handle(msg: message.Message, agent_gvh: Gvh):
    """
    handle variable message
    :param msg: message to be handled
    :param agent_gvh: agent gvh
    """
    var = msg.content
    updater = msg.sender
    try:

        if agent_gvh.dsm[var.name].last_update(updater) is not None and agent_gvh.dsm[var.name].last_update(
                updater) > msg.timestamp:
            pass
        else:
            agent_gvh.dsm[var.name].set_val(var.get_val(updater), updater)
            agent_gvh.dsm[var.name].set_update(msg.timestamp, updater)

    except KeyError:
        print("warning: trying to update possibly undeclared variable")
        pass


message_handler = dict()

message_handler[MsgType.MUTEX_REQUEST] = base_mutex_request_handle
message_handler[MsgType.MUTEX_GRANT] = base_mutex_grant_handle
message_handler[MsgType.MUTEX_RELEASE] = base_mutex_release_handle
message_handler[MsgType.VAR_UPDATE] = message_update_handle
message_handler[MsgType.STOP_COMM] = stop_comm_msg_handle
message_handler[MsgType.INIT] = init_msg_handle
message_handler[MsgType.INIT_CONFIRM] = init_msg_confirm_handle
message_handler[MsgType.ROUND_UPDATE] = round_update_msg_handle
message_handler[MsgType.ROUND_UPDATE_CONFIRM] = round_update_msg_confirm_handle
message_handler[MsgType.STOP] = stop_msg_handle
message_handler[MsgType.STOP_CONFIRM] = stop_msg_confirm_handle
