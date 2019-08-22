from src.apps.taskwithoutgoing import TaskApp
from src.config.configs import AgentConfig, default_qc_moat_config, default_car_moat_config
from src.functionality.base_mutex_handler import BaseMutexHandler
from src.motion.rrt_drone import RRT as RRT_DRONE
from src.motion.rrt_car import RRT as RRT_CAR


bots = 4

a1 = AgentConfig(1, bots, "", rport=2001, plist=[], mh=BaseMutexHandler, is_leader=False, mhargs=[False,1])
a2 = AgentConfig(2, bots, "", rport=2001, plist=[], mh=BaseMutexHandler, is_leader=True, mhargs=[True,2])
a3 = AgentConfig(3, bots, "", rport=2001, plist=[], mh=BaseMutexHandler, is_leader=False, mhargs=[False,3])
a4 = AgentConfig(4, bots, "", rport=2001, plist=[], mh=BaseMutexHandler, is_leader=False, mhargs=[False,4])

app1 = TaskApp(a1)
