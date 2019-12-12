from src.config.configs import AgentConfig
from src.functionality.basicMutexHandler import BasicMutexHandler
from src.tests.app.addnums import AddNums

bots = 3
p = [2001,2002,2003]
a0 = AgentConfig(1, bots, "", r_port=2002, plist=p, mh=BasicMutexHandler, is_leader=False, mh_args=[False, 1])
app0 = AddNums(a0)
