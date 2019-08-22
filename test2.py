from src.apps.taskappwithpaths import TaskApp
from src.config.configs import AgentConfig, default_qc_moat_config, default_car_moat_config
from src.motion.moat_test_car import MoatTestCar
from src.motion.moat_test_drone import MoatTestDrone
from src.functionality.base_mutex_handler import BaseMutexHandler
from src.motion.rrt_drone import RRT as RRT_DRONE
from src.motion.rrt_car import RRT as RRT_CAR


bots = 1

a1 = AgentConfig(1, bots, "", rport=2001, plist=[], mh=BaseMutexHandler, is_leader=False, mhargs=[False,1], moat_class=MoatTestCar)
a2 = AgentConfig(2, bots, "", rport=2001, plist=[], mh=BaseMutexHandler, is_leader=True, mhargs=[True,2], moat_class= MoatTestDrone)
a3 = AgentConfig(3, bots, "", rport=2001, plist=[], mh=BaseMutexHandler, is_leader=False, mhargs=[False,3],moat_class=MoatTestDrone)
a4 = AgentConfig(4, bots, "", rport=2001, plist=[], mh=BaseMutexHandler, is_leader=False, mhargs=[False,4], moat_class=MoatTestCar)
m0 = default_car_moat_config('hotdec_car')
m1 = default_car_moat_config('f1car')
m2 = default_qc_moat_config('cyphyhousecopter')
m3 = default_qc_moat_config('cyphyhousecopter1')
m0.planner = RRT_CAR(goal_sample_rate=15)
m1.planner = RRT_CAR(goal_sample_rate=15)
m2.planner = RRT_DRONE()
m3.planner = RRT_DRONE()

app2 = TaskApp(a2,m3)
