#!/usr/bin/env python3

import sys

from src.apps.taskapp import TaskApp
from src.config.config_funcs import get_configs

a_c, m_c = get_configs(sys.argv[1])
print(a_c,"\n\n", m_c)
#app = TaskApp(a_c, m_c)
