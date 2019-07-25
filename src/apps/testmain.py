from src.apps.addnums import AddNums
from src.config.configs import AgentConfig
from src.functionality.base_mutex_handler import BaseMutexHandler

bots = 3

a1 = AgentConfig(1, bots, "", rport=2000, plist=[], mh=BaseMutexHandler(False, 1), is_leader=False)
a2 = AgentConfig(2, bots, "", rport=2000, plist=[], mh=BaseMutexHandler(True, 2), is_leader=True)
a3 = AgentConfig(3, bots, "", rport=2000,plist=[], mh=BaseMutexHandler(False,3), is_leader=False )
app1 = AddNums(a1)
app2 = AddNums(a2)
app3 = AddNums(a3)