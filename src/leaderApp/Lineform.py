import agentThread
import time
from time import sleep
from gvh import gvh
from threading import Lock
import dsm


class LineFormApp(agentThread.agentThread):

    def __init__(self,name,bcastqueue,barrier,dd,numbots = 1,locks = []):
        super(LineFormApp, self).__init__(gvh(name, bcastqueue,numbots),dd,locks)
        self.barrier = barrier



    def run(self):
       # print("here")
       lock1 = self.locks[0]

       candidate = self.dd.get('candidate')
       if candidate is None :
            candidate = dsm.dsmvar('candidate', -1, -1, 'int', -1)
            self.dd.add(candidate)

       numvoted = self.dd.get('numvoted')
       if numvoted is None:
           numvoted = dsm.dsmvar('numvoted',-1,0,'int',-1)
           self.dd.add(numvoted)

       leader = dsm.dsmvar('leader',-1,0,'int',-1)
       voted = False

       while not(self.stopped()):

            sleep(1)
            #self.gvh.comms.wake()
            #self.gvh.commHandler.wake()

            if not voted:
                with lock1:
                    if self.dd.get('candidate') <= self.id :
                        self.dd.put('candidate',self.id,time.time())
                    else:
                        pass
                    numvoted = self.dd.get('numvoted') + 1
                    #print(self.id,numvoted)
                    self.dd.put('numvoted',numvoted,time.time())
                    #print(self.id, self.dd.get('numvoted'))
                    voted = True
            else:
                #print("here",self.id)
                with lock1:
                    #print(self.id," has lock 1 with numVoted ",self.dd.get('numvoted'))

                    if self.dd.get('numvoted') == self.gvh.participants:
                    #if voted:
                        leader = self.dd.get('candidate')
                        #print("candidate: ",self.dd.get('candidate'))
                        print("stopping",self.name)
                        print('leader',leader)
                        self.stop()


            self.barrier.wait()
            #print(list(self.gvh.comms.bcastqueue))

