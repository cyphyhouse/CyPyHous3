import agentThread
import time
from time import sleep
from gvh import gvh
from threading import RLock,Barrier
import dsm
import motion


class LineFormApp(agentThread.agentThread):

    def __init__(self,name,bcastqueue,barrier,dd,numbots = 1,locks = []):
        super(LineFormApp, self).__init__(gvh(name, bcastqueue,numbots),dd,locks)
        self.barrier = barrier


    def run(self):
       # print("here")
       print(self.gvh.moat.target)
       lock1 = self.locks[0]
       pid = self.gvh.pid()

       pos = self.dd.get('pos',pid)
       if pos is None:
            pos = dsm.dsmvar('pos', pid, -1, ('int','ar'), -1)
            self.dd.add(pos)


       while not(self.stopped()):
            #print("pid",pid)

            sleep(1)

            if pid == 0 or pid == 3:
                self.dd.put('pos',self.gvh.moat.state.position,time.time(),pid)
                #print("testing",self.dd.get('pos',pid))
                #print(list(self.dd.varmap.keys()))
                #self.stop()
                pass
                #continue
            else:
                #print(pid," requesting lock")
                with lock1:
                    #print(pid," has lock")
                    #print (self.dd)
                    if self.dd.get('pos',pid+1) is None or self.dd.get('pos',pid-1) is None:
                        #print("here")
                        pass
                    else:
                        #print("herenow")
                        pass
                    self.dd.put('pos',self.gvh.moat.state.position,time.time(),pid)


                #print(pid,"releasing lock")
            #try:
            print("here",pid)
            self.barrier.wait()
            #except:
            #    continue
            #print(list(self.gvh.comms.bcastqueue))

