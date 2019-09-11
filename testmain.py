import sys

from src.apps.car_follow import BasicFollowApp
from src.config.config_funcs import get_configs

a_c, m_c = get_configs(sys.argv[1])
app = BasicFollowApp(a_c, m_c)
