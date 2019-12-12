# Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

import time

import src.harness.msg_create as mc
import src.objects.message as message
from src.datatypes.message_types import MsgType
from src.harness.gvh import Gvh


# ------------ ROUND SYNC METHODS --------------


def round_update_msg_handle(msg: message.Message, agent_gvh: Gvh):
    """
    message handling for round update message

    :param msg: message to be handled
    :type msg: Message

    :param agent_gvh: agent gvh
    :type agent_gvh: Gvh
    """
    rounding, round_num = msg.sender, msg.content
    if agent_gvh.is_leader:
        if rounding in agent_gvh.round_counter:
            pass
        else:
            agent_gvh.round_counter.append(rounding)
        leader_id = -1
        if agent_gvh.is_leader:
            leader_id = agent_gvh.pid

        msg1 = mc.round_update_msg_confirm_create(agent_gvh.pid, leader_id, agent_gvh.round_num, time.time())
        if len(agent_gvh.round_counter) == agent_gvh.n_sys:
            agent_gvh.send(msg1)


def round_update_msg_confirm_handle(msg: message.Message, agent_gvh: Gvh):
    """
    handle round update confirm message

    :param msg: message to be handled
    :type msg: Message

    :param agent_gvh: agent gvh
    :type agent_gvh: Gvh
    """
    leader_id, round_num = int(msg.content[0]), int(msg.content[1])
    if round_num < agent_gvh.round_num:
        pass
    elif msg.sender == leader_id:
        agent_gvh.update_round = True
        agent_gvh.round_num = round_num + 1
        agent_gvh.start_time = time.time()
        agent_gvh.round_counter = []


# ------------ GRACEFUL EXIT METHODS --------------


def stop_msg_handle(msg: message.Message, agent_gvh: Gvh):
    """
    message handler for stop message

    :param msg: message to be handled
    :type msg: Message

    :param agent_gvh: agent gvh
    :type agent_gvh: Gvh
    """
    stopping = msg.sender
    if agent_gvh.is_leader:

        if stopping in agent_gvh.stop_counter:
            pass
        else:
            agent_gvh.stop_counter.append(stopping)
        leader_id = -1
        if agent_gvh.is_leader:
            leader_id = agent_gvh.pid

        msg1 = mc.stop_msg_confirm_create(agent_gvh.pid, leader_id, agent_gvh.round_num)
        if len(agent_gvh.stop_counter) == agent_gvh.n_sys:
            agent_gvh.send(msg1)


def stop_msg_confirm_handle(msg: message.Message, agent_gvh: Gvh):
    """
    confirm stop message confirmation

    :param msg: message to be handled
    :type msg: Message

    :param agent_gvh: agent gvh
    :type agent_gvh: Gvh
    """
    leader_id = int(msg.content)
    if int(msg.sender) == leader_id:
        agent_gvh.is_alive = False


# ------------ SYNCHRONIZED INITIALIZATION METHODS --------------


def init_msg_handle(msg: message.Message, agent_gvh: Gvh):
    """
    init message handler
    :param msg: message to be handled
    :type msg: Message

    :param agent_gvh: agent gvh
    :type agent_gvh: Gvh

    """
    initializing = msg.sender
    if agent_gvh.is_leader:
        if initializing in agent_gvh.init_counter:
            pass
        else:
            agent_gvh.init_counter.append(initializing)
        leader_id = -1
        if agent_gvh.is_leader:
            leader_id = agent_gvh.pid
        msg1 = mc.init_msg_confirm_create(agent_gvh.pid, leader_id, agent_gvh.round_num)
        if len(agent_gvh.init_counter) == agent_gvh.n_sys:
            agent_gvh.send(msg1)


def init_msg_confirm_handle(msg: message.Message, agent_gvh: Gvh):
    """
    handle init confirmation message

    :param msg: message to be handled
    :type msg: Message

    :param agent_gvh: agent gvh
    :type agent_gvh: Gvh
    """
    if msg.sender == msg.content:
        agent_gvh.init = True
        agent_gvh.start_time = time.time()


def stop_comm_msg_handle(msg: message.Message, agent_gvh: Gvh) -> None:
    """
    dummy message to stop listening port.
    """
    pass


# ------------ BASIC MUTEX MANAGEMENT METHODS --------------


def basic_mutex_request_handle(msg: message.Message, agent_gvh: Gvh) -> None:
    """
    add request to list of requests

    :param msg: message to be handled
    :type msg: Message

    :param agent_gvh: agent gvh
    :type agent_gvh: Gvh

    """
    mutex_id, req_num = msg.content
    requester = msg.sender
    if agent_gvh.is_leader:
        agent_gvh.mutex_handler.add_request(mutex_id, requester, req_num)
    else:
        pass


def basic_mutex_grant_handle(msg: message.Message, agent_gvh: Gvh) -> None:
    """
    grant first request

    :param msg: message to be handled
    :type msg: Message

    :param agent_gvh: agent gvh
    :type agent_gvh: Gvh

    """
    mutex_id, grantee, mutex_num = msg.content
    try:
        agent_gvh.mutex_handler.mutexes[mutex_id].mutex_holder = grantee
    except KeyError:
        print("tried granting possibly undeclared mutex")


def basic_mutex_release_handle(msg: message.Message, agent_gvh: Gvh) -> None:
    """
    release mutex held

    :param msg: message to be handled
    :type msg: Message

    :param agent_gvh: agent gvh
    :type agent_gvh: Gvh

    """
    mutex_id = msg.content
    releasing = msg.sender
    if agent_gvh.is_leader:
        try:
            if agent_gvh.mutex_handler.mutexes[mutex_id].mutex_holder == releasing:
                agent_gvh.mutex_handler.mutexes[mutex_id].mutex_holder = None

        except KeyError:
            print("tried releasing possibly undeclared mutex")
    else:
        pass


# ------------ SHARED VARIABLE METHODS --------------


def message_update_handle(msg: message.Message, agent_gvh: Gvh):
    """
    handle variable message

    :param msg: message to be handled
    :type msg: Message

    :param agent_gvh: agent gvh
    :type agent_gvh: Gvh

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

message_handler[MsgType.MUTEX_REQUEST] = basic_mutex_request_handle
message_handler[MsgType.MUTEX_GRANT] = basic_mutex_grant_handle
message_handler[MsgType.MUTEX_RELEASE] = basic_mutex_release_handle
message_handler[MsgType.VAR_UPDATE] = message_update_handle
message_handler[MsgType.STOP_COMM] = stop_comm_msg_handle
message_handler[MsgType.INIT] = init_msg_handle
message_handler[MsgType.INIT_CONFIRM] = init_msg_confirm_handle
message_handler[MsgType.ROUND_UPDATE] = round_update_msg_handle
message_handler[MsgType.ROUND_UPDATE_CONFIRM] = round_update_msg_confirm_handle
message_handler[MsgType.STOP] = stop_msg_handle
message_handler[MsgType.STOP_CONFIRM] = stop_msg_confirm_handle
