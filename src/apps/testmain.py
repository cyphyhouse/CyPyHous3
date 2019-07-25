from src.apps.addnums import AddNums
from src.config.configs import AgentConfig
from src.functionality.base_mutex_handler import BaseMutexHandler

bots = 2

a1 = AgentConfig(1, bots, "", rport=2000, plist=[], mh=BaseMutexHandler(False, 1), is_leader=False)
a2 = AgentConfig(2, bots, "", rport=2000, plist=[], mh=BaseMutexHandler(True, 2), is_leader=True)
app1 = AddNums(a1)
#app2 = AddNums(a2)