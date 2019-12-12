from src.config.configs import AgentConfig
from src.functionality.basicMutexHandler import BasicMutexHandler
from src.tests.app.addnums import AddNums

bots = 3
p = [2001,2002,2003]
a0 = AgentConfig(0, bots, "", r_port=2001, plist=p, mh=BasicMutexHandler, is_leader=True, mh_args=[True, 0])
app0 = AddNums(a0)
