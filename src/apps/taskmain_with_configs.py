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

from src.apps.taskapp import TaskApp
from src.config.configs import AgentConfig, default_car_moat_config, default_qc_moat_config
from src.functionality.base_mutex_handler import BaseMutexHandler
from src.motion.rrt_star import RRT as RRT_CAR
from src.motion.rrt_star_drone import RRT as RRT_DRONE
from src.motion.simpleplanner import SimplePlanner
from src.motion.reeds_shepp_planner import Reeds_Shepp_Planner
from src.motion.moat_test_drone import MoatTestDrone
from src.motion.moat_test_car import MoatTestCar

bots = 2
mhargs = [[True,0], [False, 1], [False,2], [False,3]]
plist = []
r_port = 2000
r_ip = ""
pid = [0,1,2,3]


moatcar = MoatTestCar
moatdrone = MoatTestDrone

a0 = AgentConfig(pid[0], bots, r_ip, r_port, plist, BaseMutexHandler, mhargs=mhargs[0], is_leader=mhargs[0][0])
a1 = AgentConfig(pid[1], bots, r_ip, r_port, plist, BaseMutexHandler, mhargs=mhargs[1], is_leader=mhargs[1][0])
a2 = AgentConfig(pid[2], bots, r_ip, r_port, plist, BaseMutexHandler, mhargs=mhargs[2], is_leader=mhargs[2][0])
a3 = AgentConfig(pid[3], bots, r_ip, r_port, plist, BaseMutexHandler, mhargs=mhargs[3], is_leader=mhargs[3][0])

m0 = default_car_moat_config('hotdec_car')
m1 = default_car_moat_config('f1car')
m2 = default_qc_moat_config('cyphyhousecopter')
m3 = default_qc_moat_config('cyphyhousecopter1')
m0.planner = Reeds_Shepp_Planner()
m1.planner = RRT_CAR(goal_sample_rate=15)
m2.planner = RRT_DRONE()
m3.planner = RRT_DRONE()

#change this line
a, m = a0, m0
a.moat_class = MoatTestCar
app = TaskApp(a,m)

