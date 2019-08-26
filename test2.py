from src.apps.addnums import AddNums
from src.apps.taskwithpaths import TaskApp
from src.config.configs import AgentConfig
from src.functionality.base_mutex_handler import BaseMutexHandler

bots = 4

a1 = AgentConfig(1, bots, "", rport=2001, plist=[2001,2002,2003,2004], mh=BaseMutexHandler, is_leader=False, mhargs=[False,1])
a2 = AgentConfig(2, bots, "", rport=2002, plist=[2001,2002,2003,2004], mh=BaseMutexHandler, is_leader=True, mhargs=[True,2])
a3 = AgentConfig(3, bots, "", rport=2003, plist=[2001,2002,2003,2004], mh=BaseMutexHandler, is_leader=False, mhargs=[False,3])
a4 = AgentConfig(4, bots, "", rport=2004, plist=[2001,2002,2003,2004], mh=BaseMutexHandler, is_leader=False, mhargs=[False,4])
#app2 = AddNums(a2)
app2 = TaskApp(a2)
