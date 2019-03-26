import testApp,testApp1
import dsm
from threading import Barrier,RLock
import queue
locks = [RLock()]
numRobots = 3
dd = dsm.dsm()
b = Barrier(numRobots)
for i in range(numRobots):
    bcastqueue = []
    app = testApp.TestApp("robot"+str(i),bcastqueue,b,dd,numRobots,locks)
    #app = testApp1.TestApp1("robot"+str(i),bcastqueue,b,dd,numRobots,locks)



