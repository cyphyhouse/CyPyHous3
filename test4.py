from src.apps.addnums import AddNums
from src.apps.taskwithpaths import TaskApp
from src.config.configs import AgentConfig
from src.functionality.base_mutex_handler import BaseMutexHandler

bots = 3
p = [2001,2002,2003,2004,2005]
a0 = AgentConfig(0, bots, "", rport=2001, plist= p, mh=BaseMutexHandler, is_leader=False, mhargs=[False,0])
a1 = AgentConfig(1, bots, "", rport=2002, plist= p, mh=BaseMutexHandler, is_leader=False, mhargs=[False,1])
a2 = AgentConfig(0, bots, "", rport=2003, plist= p, mh=BaseMutexHandler, is_leader=True, mhargs=[True,0])
a3 = AgentConfig(3, bots, "", rport=2004, plist= p, mh=BaseMutexHandler, is_leader=False, mhargs=[False,3])
a4 = AgentConfig(4, bots, "", rport=2005, plist=p, mh=BaseMutexHandler, is_leader=False, mhargs=[False,4])
app3 = AddNums(a3)
#app3 = TaskApp(a3)
