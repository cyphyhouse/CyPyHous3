from agentThread import AgentThread
import dsm
import time
from time import sleep
from gvh import gvh
from point import *


class BrakingApp(AgentThread):

    def __init__(self, name, barrier, numbots=1, sim=True, dd=None, locks=[]):
        super(BrakingApp, self).__init__(gvh(name, 'default', numbots, sim,'Timed'), dd, locks)
        self.__barrier = barrier


    def run(self):
       n = 0
       j = 0
       mode = 0
       safedist = 10
       lock0 = self.locks[0]
       numBots = self.gvh.participants
       pid = self.gvh.pid
       pos = self.dd.get('pos',pid)
       if pos is None:
            pos = dsm.dsmvar('pos', pid, self.gvh.moat.state.pos , ('point','ar'), -1)
            self.dd.add(pos)

       while not(self.stopped()):
            time.sleep(.01)
            n = n + 1

            if pid == numBots - 1:

                if mode == 0:
                    if j >= 20:
                         mode = 1
                else:
                    if j <= 0:
                        mode = 0

                if mode == 0:
                    self.gvh.moat.state.xacc = 2
                    print("cruising")
                    j = j + 1

                if  mode == 1:
                    self.gvh.moat.state.xacc = -2
                    print("slow")
                    j = j - 1




                self.dd.put('pos',self.gvh.moat.state.pos,time.time(),pid)

            else:

                if self.dd.get('pos',pid+1) is None:
                    pass

                else:

                    dist = self.dd.get('pos',pid+1).x - self.dd.get('pos',pid).x
                    if dist <= 0:
                        self.stop()
                        continue

                    if dist >= safedist:
                        self.gvh.moat.state.xacc = 1
                    else:
                        self.gvh.moat.state.xacc = -1
                    print(pid,dist,self.gvh.moat.state.xacc,self.gvh.moat.state.xvel)
                self.dd.put('pos',self.gvh.moat.state.pos,time.time(),pid)
            #print(n)
            if n == 30:
                print("stopping",pid)
                self.stop()
