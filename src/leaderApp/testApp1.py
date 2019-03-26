import agentThread
import time
from time import sleep
from gvh import gvh
from threading import RLock
import dsm


class TestApp1(agentThread.agentThread):

    def __init__(self,name,bcastqueue,barrier,dd,numbots = 1,locks = []):
        super(TestApp1, self).__init__(gvh(name, bcastqueue,numbots),dd,locks)
        self.barrier = barrier

    def run(self):
       # print("here")
       lock1 = self.locks[0]
       sum = dsm.dsmvar('sum', -1, 0, 'int', -1)
       numAdded = dsm.dsmvar('numAdded',-1,0,'int',-1)
       self.dd.add(sum)
       self.dd.add(numAdded)
       added = False

       while not(self.stopped()):

            sleep(0.8)
            #self.gvh.comms.wake()
            #self.gvh.commHandler.wake()
            #if (True):
            if not added:
                #with lock1:
                if self.dd.get('numAdded') <= self.id :
                    sum = self.dd.get('sum')
                    self.dd.put('sum',sum + self.id * 2,time.time())
                else:
                    pass
                numAdded = self.dd.get('numAdded') + 1
                print(self.id,numAdded)
                self.dd.put('numAdded',numAdded,time.time())
                print(self.id, self.dd.get('numAdded'))
                added = True
            #if (True):
            else:
                #with lock1:
                if self.dd.get('numAdded') >= self.gvh.participants and added:
                #if added:
                    print("sum: ",self.dd.get('sum'))
                    print("stopping",self.name)
                    self.stop()


            self.barrier.wait()
            #print(list(self.gvh.comms.bcastqueue))

