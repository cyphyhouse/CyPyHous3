from src.apps.task import Addnums
from src.config.config_funcs import get_configs

config = get_configs("irl.local.yml")

Addnums(config[0], config[1])
