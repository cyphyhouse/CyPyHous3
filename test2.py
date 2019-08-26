from src.apps.addnums import AddNums
from src.apps.taskwithpaths import TaskApp
from src.config.configs import AgentConfig
from src.functionality.base_mutex_handler import BaseMutexHandler

bots = 3

a0 = AgentConfig(0, bots, "", rport=2001, plist=[2001,2002,2003,2004,2005], mh=BaseMutexHandler, is_leader=False, mhargs=[False,0])
a1 = AgentConfig(1, bots, "", rport=2002, plist=[2001,2002,2003,2004,2005], mh=BaseMutexHandler, is_leader=True, mhargs=[True,1])
a2 = AgentConfig(2, bots, "", rport=2003, plist=[2001,2002,2003,2004,2005], mh=BaseMutexHandler, is_leader=True, mhargs=[True,2])
a3 = AgentConfig(3, bots, "", rport=2004, plist=[2001,2002,2003,2004,2005], mh=BaseMutexHandler, is_leader=False, mhargs=[False,3])
a4 = AgentConfig(4, bots, "", rport=2005, plist=[2001,2002,2003,2004,2005], mh=BaseMutexHandler, is_leader=False, mhargs=[False,4])
app1 = AddNums(a1)
#app1 = TaskApp(a1)
