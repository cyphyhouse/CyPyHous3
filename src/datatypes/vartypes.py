# Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

from enum import Enum, unique


@unique
class ShareType(Enum):
    """
    add more variable sharing options as needed here
    """
    ALL_WRITE = 1
    ALL_READ = 2
