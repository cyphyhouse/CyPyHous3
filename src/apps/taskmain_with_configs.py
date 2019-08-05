import sys

from src.config.config_funcs import get_configs
from src.config.configs import default_car_moat_config
from src.apps.taskapp import TaskApp
from src.motion.rrt_star import RRT
from src.apps.taskapp import TaskApp
from src.config.configs import AgentConfig, default_car_moat_config, default_qc_moat_config
from src.functionality.base_mutex_handler import BaseMutexHandler
from src.motion.rrt_star import RRT as RRT_CAR
from src.motion.rrt_star_drone import RRT as RRT_DRONE
from src.motion.simpleplanner import SimplePlanner
from src.motion.reeds_shepp_planner import Reeds_Shepp_Planner
from src.motion.moat_test_drone import MoatTestDrone
from src.motion.moat_test_car import MoatTestCar

bots = 1
mhargs = [[True,0], [False, 1], [False,2], [False,3]]
plist = []
r_port = 2000
r_ip = ""
pid = [0,1,2,3]


moatcar = MoatTestCar
moatdrone = MoatTestDrone
a0 = AgentConfig(pid[0], bots, r_ip, r_port, plist, BaseMutexHandler, mhargs=mhargs[0], is_leader=mhargs[0][0])
a1, m1 = get_configs(config_filename=sys.argv[1])
m0 = default_car_moat_config('hotdec_car')
m0.planner = RRT
a0.moat_class = MoatTestCar
print( a0 == a1)
# print(m1.pos_msg_type)
#TaskApp(a1, m1)
