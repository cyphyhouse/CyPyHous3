from src.config.configs import AgentConfig
from src.functionality.base_mutex_handler import BaseMutexHandler
from src.tests.app.addnums import AddNums

bots = 3
p = [2001,2002,2003]
a0 = AgentConfig(2, bots, "", rport=2003, plist=p, mh=BaseMutexHandler, is_leader=False, mhargs=[False, 2])
app0 = AddNums(a0)
