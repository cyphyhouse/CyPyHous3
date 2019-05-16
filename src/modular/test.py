import Lineform,BrakingApp,TaskApp,Shapeform
import dsm
import queue
from threading import RLock,Barrier
locks = [RLock(),RLock()]
lockqueue = queue.Queue()
numRobots = 4
dd = dsm.dsm(numRobots)
dd1 = dsm.dsm(numRobots*numRobots)
b = Barrier(numRobots,timeout=2)
b1 = Barrier(numRobots*numRobots,timeout=12)
for i in range(numRobots):

    app = Lineform.LineFormApp(i,b,numRobots,True,dd,locks)

    #for j in range(numRobots):
    #    app = Shapeform.ShapeformApp(i,b1,numRobots*numRobots,True,dd1,locks,j)

    #app = TaskApp.TaskApp(i,b,numRobots,True,dd,locks)
    #app = BrakingApp.BrakingApp(i,b,numRobots,True,dd,locks)

