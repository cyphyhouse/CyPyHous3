#!/usr/bin/env python3

import sys
import yaml

from src.apps.taskapp import TaskApp
from src.config.config_funcs import get_configs

with open(sys.argv[1], 'r') as f:
    cfg = yaml.safe_load(f)
    a_c, m_c = get_configs(cfg)
app = TaskApp(a_c, m_c)
