import Lineform
import dsm
from threading import RLock,Barrier
locks = [RLock(),RLock()]
numRobots = 4
dd = dsm.dsm(numRobots)
b = Barrier(numRobots,timeout=2)
for i in range(numRobots):
    app = Lineform.LineFormApp("robot"+str(i),b,numRobots,True,dd,locks)


