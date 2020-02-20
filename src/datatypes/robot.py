# Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

from enum import Enum, unique


@unique
class BotType(Enum):
    """
    add more bot types as needed here
    """
    QUAD = 1
    CAR = 2
