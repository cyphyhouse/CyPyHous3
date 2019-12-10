# Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

from src.datatypes.message_types import MsgType as mt
from src.objects.message import Message


def round_update_msg_create(pid: int, round_num: int, ts: float) -> Message:
    """
    create to update round
    :param pid: agent pid that is ready to update round
    :param round_num: round number
    :param ts: time stamp
    :return: round update message
    """
    return Message(pid, mt.ROUND_UPDATE, round_num, ts)


def round_update_msg_confirm_create(pid: int, leaderid: int, round_num:int, ts: float) -> Message:
    """
    create to confirm round udpate
    :param pid: agent pid
    :param leaderid: leader pid
    :param round_num: round number
    :param ts: time stamp
    :return: round update confirmation message
    """
    return Message(pid, mt.ROUND_UPDATE_CONFIRM, (leaderid,round_num) , ts)


def stop_msg_create(pid: int, round_num: int, ts: float) -> Message:
    """
    create message to stop
    :param pid: agent pid
    :param round_num: round number
    :param ts: time stamp
    :return: stop message creation
    """
    return Message(pid, mt.STOP, round_num, ts)


def stop_msg_confirm_create(pid: int, leaderid: int, ts: float) -> Message:
    """
    create to confirm stop
    :param pid: agent pid
    :param leaderid: leader pid
    :param ts: time stamp
    :return: stop message confirmation create
    """
    return Message(pid, mt.STOP_CONFIRM, leaderid, ts)


def init_msg_create(pid: int, ts: float) -> Message:
    """
    create init message
    :param pid: pid
    :param ts:
    :return: init message
    """
    return Message(pid, mt.INIT, None, ts)


def init_msg_confirm_create(pid: int, leaderid: int, ts: float) -> Message:
    """
    create init confirmation message
    :param pid:
    :param leaderid:
    :param ts:
    :return: init confirmation message
    """
    return Message(pid, mt.INIT_CONFIRM, leaderid, ts)


def stop_comm_msg_create(pid: int, ts: float) -> Message:
    """
    create a message to stop the comm_handler
    :param pid:
    :param ts:
    :return: stop communication handler message
    """
    return Message(pid, mt.STOP_COMM, None, ts)


def mutex_request_create(mutex_id: int, req_num: int, pid: int, ts: float) -> Message:
    """
    create mutex request message
    :param mutex_id: variable to create mutex request for
    :param pid: pid of agent requesting mutex
    :param req_num : request number
    :param ts: time stamp
    :return: mutex request message
    """
    contents = (mutex_id, req_num)
    return Message(pid, mt.MUTEX_REQUEST, contents, ts)


def mutex_grant_create(mutex_id: int, agent_id: int, pid: int, mutexnum: int, ts: float) -> Message:
    """
    grant mutex creation message
    :param mutexnum :mutex number
    :param mutex_id: variable to grant mutex on
    :param agent_id: agent to grant mutex to
    :param pid: leader pid who is granting the mutex
    :param ts: timestamp
    :return: mutex grant message
    """
    contents = (mutex_id, agent_id, mutexnum)
    return Message(pid, mt.MUTEX_GRANT, contents, ts)


def mutex_release_create(mutex_id: int, pid: int, ts: float) -> Message:
    """
    create release held mutex message
    :param mutex_id: variable name
    :param pid: releasing agent's pid
    :param ts: timestamp
    :return: mutex release
    """
    contents = mutex_id
    return Message(pid, mt.MUTEX_RELEASE, contents, ts)

def dsm_update_create(pid: int, dsmvar_updated, owner, roundnum):
    """
    create dsm update message.
    :param pid : agent pid
    :param dsmvar_updated : dsmvar to be updated
    :param owner: share type
    :param roundnum: round number of udpate
    """
    return Message(pid, mt.VAR_UPDATE, dsmvar_updated, roundnum)


