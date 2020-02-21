# Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

from enum import Enum, unique


@unique
class MsgType(Enum):
    """
    add more message types as needed here
    """
    INIT = 0
    INIT_CONFIRM = 1
    STOP = 3
    STOP_COMM = 4
    STOP_CONFIRM = 5
    MUTEX_REQUEST = 6
    MUTEX_GRANT = 7
    MUTEX_RELEASE = 8
    ROUND_UPDATE = 9
    ROUND_UPDATE_CONFIRM = 10
    VAR_UPDATE = 11
