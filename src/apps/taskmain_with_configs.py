import sys

from src.config.config_funcs import get_configs
from src.config.configs import default_car_moat_config

a1, m1 = get_configs(config_filename=sys.argv[1])
m0 = default_car_moat_config('hotdec_car')

print(m0, m1)
# print(m1.pos_msg_type)
# TaskApp(a1, m1)
