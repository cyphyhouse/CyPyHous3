from typing import Any

import message
import datatypes
from gvh import Gvh


def update_create(data_type: datatypes.dtypes, pid: int, var: str, value: Any, ts: float) -> message.Message:
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
    if msg.m_type > 0 and msg.m_type <=2 :
        varname, value = msg.content.split(",")
        agent_gvh.agent_dsm.put(msg.sender, varname, update_type[msg.m_type] (value))

update_message_type = dict()
update_message_type[datatypes.dtypes.INT] = 1
update_message_type[datatypes.dtypes.BOOL] = 2

update_type = dict()
update_type[1] = int
update_type[2] = bool


message_handler = dict()
message_handler[1] = update_handle
message_handler[2] = update_handle

