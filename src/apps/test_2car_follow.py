from src.apps.two_car_follow_path import BasicFollowApp
from src.functionality.base_mutex_handler import BaseMutexHandler
from src.config.configs import AgentConfig, default_car_moat_config
from src.motion.rrt_star import RRT

m = default_car_moat_config('f1car')
m.planner = RRT(goal_sample_rate=20)
a = AgentConfig(1, 2, "", 2000, mh=BaseMutexHandler(False,1))
app = BasicFollowApp(a, m)


m = default_car_moat_config('hotdec_car')
m.planner = RRT(goal_sample_rate=20)
a = AgentConfig(0, 2, "", 2000, is_leader=True, mh= BaseMutexHandler(True,0))
app = BasicFollowApp(a, m)

