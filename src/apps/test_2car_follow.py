from src.apps.addnums import AddNums
from src.config.configs import AgentConfig
from src.functionality.base_mutex_handler import BaseMutexHandler

bots = 2

a = AgentConfig(1, bots, "", 2000, [], BaseMutexHandler(False, 1), is_leader=False)
a = AgentConfig(2, bots, "", 2000, [], BaseMutexHandler(True, 2), is_leader=True)
app = AddNums(a)
