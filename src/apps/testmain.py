from src.apps.addnums import AddNums
from src.config.configs import AgentConfig
from src.functionality.base_mutex_handler import BaseMutexHandler

bots = 2

#a = AgentConfig(1, bots, "", rport=2000,plist=[], mh=BaseMutexHandler(False, 1))
a = AgentConfig(2, bots, "", rport=2000,plist=[], mh=BaseMutexHandler(True, 2))
app = AddNums(a)

