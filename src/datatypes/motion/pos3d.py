#  Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

from src.datatypes.motion.pos import Pos


class pos3d(Pos):
    """
    object used in Koord.
    """

    def __init__(self, x: float, y: float, z: float):
        super(pos3d, self).__init__(np.array([x, y, z]))
