from src.config.configs import AgentConfig
from src.functionality.base_mutex_handler import BaseMutexHandler
from src.tests.app.addnums import AddNums

bots = 3
p = [2001,2002,2003]
a0 = AgentConfig(0, bots, "", rport=2001, plist=p, mh=BaseMutexHandler, is_leader=True, mhargs=[True, 0])
app0 = AddNums(a0)
