import sys

from src.config.config_funcs import get_configs
from src.config.configs import default_car_moat_config
from src.apps.taskapp import TaskApp
from src.motion.rrt_star import RRT

a1, m1 = get_configs(config_filename=sys.argv[1])
m0 = default_car_moat_config('hotdec_car')
m0.planner = RRT
print(m0.planner == m1.planner)
# print(m1.pos_msg_type)
#TaskApp(a1, m1)
