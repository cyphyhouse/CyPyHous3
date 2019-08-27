#!/usr/bin/env python3

import sys

from src.apps.taskwithpaths import TaskApp
from src.config.config_funcs import get_configs

a_c, m_c = get_configs(sys.argv[1])
app = TaskApp(a_c, m_c)
