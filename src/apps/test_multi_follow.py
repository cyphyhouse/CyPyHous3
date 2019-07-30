from src.apps.multi_vehicle_follow_path import BasicFollowApp
from src.config.configs import AgentConfig, default_car_moat_config
from src.functionality.base_mutex_handler import BaseMutexHandler
from src.motion.rrt_star import RRT as RRT_CAR
from src.motion.rrt_star_drone import RRT as RRT_DRONE


bots = 1
mharg

a1 = AgentConfig(0, bots, "", 2000, [], BaseMutexHandler, , mhargs=[False,0])
a2 = AgentConfig(1, bots, "", 2000, [], BaseMutexHandler, is_leader=True, mhargs=[True, 1])
m1 = default_car_moat_config('hotdec_car')
m2 = default_car_moat_config('f1car')
m2.planner = RRT_CAR(goal_sample_rate=30)
app1 = BasicFollowApp(a1, m1)
app2 = BasicFollowApp(a2, m2)
