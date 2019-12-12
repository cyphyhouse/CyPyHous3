#  Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

import time

import numpy as np

from src.datatypes.motion.cylobs import CylObs
from src.datatypes.motion.pos import Pos
from src.motion.planning import rrt_car

a = rrt_car.RRT()
p1 = Pos(np.array([-2, 0, 0]))
p2 = Pos(np.array([2, 0, 0]))

o1 = CylObs(Pos(np.array([0, 0, 0])), 1.0)

loops = 100
start_time = time.time()
for i in range(loops):
    p = a.find_path(p1, p2, [o1])
elapsed_time = time.time() - start_time
print(elapsed_time / loops)

# print(p)
# for i in range(len(p)):
# print(p[i])

# ps = a.path_smoothing([o1], p, 100)
# for i in range(len(ps)):
# print(ps[i])
