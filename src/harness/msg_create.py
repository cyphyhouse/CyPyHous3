# Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

import src.datatypes.message_types as mt
import src.objects.message as message


def round_update_msg_create(pid: int, round_num: int, ts: float) -> message.Message:
    """
    create message to update round

    :param pid: agent pid that is ready to update round
    :type pid: int

    :param round_num: round number
    :type round_num: int

    :param ts: time stamp
    :type ts: float

    :return: round update message
    :rtype: Message
    """
    return message.Message(pid, mt.MsgType.ROUND_UPDATE, round_num, ts)


def round_update_msg_confirm_create(pid: int, leader_id: int, round_num: int, ts: float) -> message.Message:
    """
    create message to confirm round update

    :param pid: agent pid
    :type pid: int

    :param leader_id: leader pid
    :type leader_id: int

    :param round_num: round number
    :type round_num: int

    :param ts: time stamp
    :type ts: float

    :return: round update confirmation message
    :rtype: Message
    """
    return message.Message(pid, mt.MsgType.ROUND_UPDATE_CONFIRM, (leader_id, round_num), ts)


def stop_msg_create(pid: int, round_num: int, ts: float) -> message.Message:
    """
    create message to stop

    :param pid: agent pid
    :type pid: int

    :param round_num: round number
    :type round_num: int

    :param ts: time stamp
    :type ts: float

    :return: stop message creation
    :rtype: Message
    """
    return message.Message(pid, mt.MsgType.STOP, round_num, ts)


def stop_msg_confirm_create(pid: int, leader_id: int, ts: float) -> message.Message:
    """
    create to confirm stop

    :param pid: agent pid
    :type pid: int

    :param leader_id: leader pid
    :type leader_id: int

    :param ts: time stamp
    :type ts: float

    :return: stop message confirmation create
    :rtype: Message
    """
    return message.Message(pid, mt.MsgType.STOP_CONFIRM, leader_id, ts)


def init_msg_create(pid: int, ts: float) -> message.Message:
    """
    create init message

    :param pid: agent pid
    :type pid: int

    :param ts: time stamp
    :type ts: float

    :return: init message
    :rtype: Message
    """
    return message.Message(pid, mt.MsgType.INIT, None, ts)


def init_msg_confirm_create(pid: int, leader_id: int, ts: float) -> message.Message:
    """
    create init confirmation message

    :param pid: agent pid
    :type pid: int

    :param leader_id: leader pid
    :type leader_id: int

    :param ts: timestamp
    :type ts: float

    :return: init confirmation message
    :rtype: Message
    """
    return message.Message(pid, mt.MsgType.INIT_CONFIRM, leader_id, ts)


def stop_comm_msg_create(pid: int, ts: float) -> message.Message:
    """
    create a message to stop the comm_handler

    :param pid: agent pid
    :type pid: int

    :param ts: timestamp
    :type ts: float

    :return: stop communication handler message
    :rtype: Message
    """
    return message.Message(pid, mt.MsgType.STOP_COMM, None, ts)


def mutex_request_create(mutex_id: str, req_num: int, pid: int, ts: float) -> message.Message:
    """
    create mutex request message

    :param mutex_id: variable to create mutex request for
    :type mutex_id: str

    :param pid: pid of agent requesting mutex
    :type pid: int

    :param req_num: request number
    :type req_num: int

    :param ts: time stamp
    :type ts: float

    :return: mutex request message
    :rtype: Message
    """
    contents = (mutex_id, req_num)
    return message.Message(pid, mt.MsgType.MUTEX_REQUEST, contents, ts)


def mutex_grant_create(mutex_id: str, agent_id: int, pid: int, mutex_num: int, ts: float) -> message.Message:
    """
    grant mutex creation message

    :param mutex_num: mutex number
    :type mutex_num: int

    :param mutex_id: variable to grant mutex on
    :type mutex_id: str

    :param agent_id: agent to grant mutex to
    :type agent_id: int

    :param pid: leader pid who is granting the mutex
    :type pid: int

    :param ts: timestamp
    :type ts: float

    :return: mutex grant message
    :rtype: Message
    """
    contents = (mutex_id, agent_id, mutex_num)
    return message.Message(pid, mt.MsgType.MUTEX_GRANT, contents, ts)


def mutex_release_create(mutex_id: str, pid: int, ts: float) -> message.Message:
    """
    create release held mutex message

    :param mutex_id: mutex name
    :type mutex_id: str

    :param pid: releasing agent's pid
    :type pid: int

    :param ts: timestamp
    :type ts: float

    :return: mutex release message
    :rtype: Message
    """
    contents = mutex_id
    return message.Message(pid, mt.MsgType.MUTEX_RELEASE, contents, ts)


def dsm_update_create(pid: int, dsm_var_updated, round_num):
    """
    create dsm update message.

    :param pid: agent pid
    :type pid: int

    :param dsm_var_updated: dsm_var to be updated
    :type dsm_var_updated: DSM

    :param round_num: round number of update
    :type round_num: int

    :return: dsm update message
    :rtype: Message
    """
    return message.Message(pid, mt.MsgType.VAR_UPDATE, dsm_var_updated, round_num)
