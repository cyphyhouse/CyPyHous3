import sys

from src.config.config_funcs import get_configs
from src.apps.taskapp import TaskApp

a1, m1 = get_configs(config_filename=sys.argv[1])

print(type(m1.pos_msg_type))
#TaskApp(a1, m1)
