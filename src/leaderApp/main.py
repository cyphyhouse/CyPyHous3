import testApp,testApp1,Lineform
import dsm
from threading import Barrier,RLock
import queue
locks = [RLock(),RLock()]
numRobots = 4
dd = dsm.dsm(numRobots)
b = Barrier(numRobots,timeout=2)
for i in range(numRobots):
    bcastqueue = []
    app = Lineform.LineFormApp("robot"+str(i),bcastqueue,b,dd,numRobots,locks)
    #ch
    #app = testApp.TestApp("robot"+str(i),bcastqueue,b,dd,numRobots,locks)
    #app = testApp1.TestApp1("robot"+str(i),bcastqueue,b,dd,numRobots,locks)



