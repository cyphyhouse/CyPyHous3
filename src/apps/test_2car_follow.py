from src.apps.two_car_follow_path import BasicFollowApp
from src.config.configs import AgentConfig, default_car_moat_config
from src.functionality.base_mutex_handler import BaseMutexHandler

bots = 2

a1 = AgentConfig(0, bots, "", 2000, [], BaseMutexHandler(False, 0), is_leader=False)
a2 = AgentConfig(1, bots, "", 2000, [], BaseMutexHandler(True, 1), is_leader=True)
m1 = default_car_moat_config('hotdec_car')
m2 = default_car_moat_config('ficar')
app1 = BasicFollowApp(a1, m1)
app2 = BasicFollowApp(a2, m2)
