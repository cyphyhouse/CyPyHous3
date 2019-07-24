from src.apps.addnums import AddNums
from src.functionality.base_mutex_handler import BaseMutexHandler
from src.config.configs import AgentConfig, default_car_moat_config

bots = 2

a = AgentConfig(1, bots, "", 2000, [], BaseMutexHandler(False, 1))
a = AgentConfig(2, bots, "", 2000, [], BaseMutexHandler(True, 2))
app = AddNums(a)

