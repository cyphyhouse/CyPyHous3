# functions for message handling. use dictionary to call them

import src.objects.message as message


def round_update_msg_create(pid, round_num, ts: float):
    return message.Message(pid, 9, round_num, ts)


def stop_msg_create(pid, round_num, ts: float):
    return message.Message(pid, 11, round_num, ts)


def stop_msg_confirm_create(pid, leaderid, ts: float):
    return message.Message(pid, 12, leaderid, ts)


def init_msg_create(pid, ts: float):
    return message.Message(pid, 7, None, ts)


def init_msg_confirm_create(pid, leaderid, ts: float):
    return message.Message(pid, 8, leaderid, ts)


def base_mutex_ack_create(pid, grantee, mutex_name, reqnum, ts: float) -> message.Message:
    msg_contents = (mutex_name, reqnum, grantee)
    return message.Message(pid, 6, msg_contents, ts)


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


def dsm_update_create(pid: int, dsmvar_updated, owner, ts):
    """
    create dsm update message. cant import from message handler because of circular imports.
    :param pid:
    :param dsmvar_updated:
    :param owner:
    :param ts:
    :return:
    """

    return message.Message(pid, 4, dsmvar_updated, ts)


def round_update_msg_confirm_create(pid, leaderid, ts: float):
    return message.Message(pid, 10, leaderid, ts)