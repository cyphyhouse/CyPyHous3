#!/usr/bin/env python3

import sys

from src.apps.taskapp import TaskApp
from src.config.config_funcs import get_configs
from src.motion.rrt_star import RRT

a_c, m_c = get_configs(sys.argv[1])
m_c.planner = RRT(goal_sample_rate=15)
app = TaskApp(a_c, m_c)
