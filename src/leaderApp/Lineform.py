import agentThread
import time
from time import sleep
from gvh import gvh
from threading import RLock,Barrier
import dsm,point
import motion
from targets import *

class LineFormApp(agentThread.agentThread):

    def __init__(self,name,bcastqueue,barrier,dd,numbots = 1,locks = []):
        super(LineFormApp, self).__init__(gvh(name, bcastqueue,numbots),dd,locks)
        self.barrier = barrier


    def run(self):
       n = 0
       # print("here")
       lock0 = self.locks[0]
       pid = self.gvh.pid()

       pos = self.dd.get('pos',pid)
       if pos is None:
            pos = dsm.dsmvar('pos', pid, self.gvh.moat.state.position , ('point','ar'), -1)
            self.dd.add(pos)

       #while (n <= 10):
            #print(n)
       while not(self.stopped()):
            #print("pid",pid)

            sleep(0.1)

            if pid == 0 or pid == 3:
                self.dd.put('pos',self.gvh.moat.state.position,time.time(),pid)
                #targets[pid] = self.gvh.moat.state.position

                pass
            else:
                with lock0:
                    #print(pid,"is at",self.gvh.moat.state.position)
                    if self.dd.get('pos',pid+1) is None or self.dd.get('pos',pid-1) is None:
                        pass
                    else:
                        point1 = self.dd.get('pos',pid+1)
                        point2 = self.dd.get('pos',pid-1)
                        target = midpoint(point1,point2)
                        print(pid,"is at",self.gvh.moat.state.position,"going to",gvh.moat.target)
                        gvh.moat.target = target

                    self.dd.put('pos',self.gvh.moat.state.position,time.time(),pid)


                #print(pid,"releasing lock")
            #try:
            #print("here",pid)
            #self.barrier.wait()
            #except:
            #    continue
            #print(list(self.gvh.comms.bcastqueue))
       print("stopping gvh")
       self.gvh.stop()




def midpoint(point1,point2):
    return point.point((point1.x + point2.x)/2,(point1.y+point2.y)/2)