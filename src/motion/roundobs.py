import numpy as np

from src.motion.obstacle import Obstacle
from src.motion.pos_types import Pos

class RoundObs(Obstacle):
    
    def __init__(self, point, radius):
        super(RoundObs, self).__init__(point, np.array([radius]))

    def __collision_point(self, point: Pos) -> bool:
        

    def __collision_path(self, path: Seg) -> bool:
        
