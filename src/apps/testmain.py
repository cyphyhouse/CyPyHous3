from src.apps.addnums import AddNums
from src.config.configs import AgentConfig
from src.functionality.base_mutex_handler import BaseMutexHandler

bots = 2

a1 = AgentConfig(1, bots, "", rport=2000, plist=[], mh=BaseMutexHandler, is_leader=False, mhargs=[False,1])
a2 = AgentConfig(2, bots, "", rport=2000, plist=[], mh=BaseMutexHandler, is_leader=True, mhargs=[True,2])
a3 = AgentConfig(3, bots, "", rport=2003, plist=[2001,2002,2003], mh=BaseMutexHandler, is_leader=False, mhargs=[False,3])

app1 = AddNums(a1)
app2 = AddNums(a2)
#app3 = AddNums(a3)
