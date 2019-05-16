from agentThread import AgentThread
import dsm
import time
from time import sleep
from gvh import gvh
from point import *


class LineFormApp(AgentThread):

    def __init__(self, name, barrier, numbots=1, sim=True, dd=None, locks=[]):
        super(LineFormApp, self).__init__(gvh(name, 'testfile', numbots, sim), dd, locks)
        self.__barrier = barrier


    def run(self):
       lock0 = self.locks[0]
       n = 0
       pid = self.gvh.pid
       pos = self.dd.get('pos',pid)
       numBots = self.gvh.participants
       if pos is None:
            pos = dsm.dsmvar('pos', pid, self.gvh.moat.state.pos , ('point','ar'), -1)
            self.dd.add(pos)

       while not(self.stopped()):
            n += 1
            sleep(0.01)
            if pid == 0 or pid == numBots -1:
                #self.gvh.moat.mode = 'Timed'
                self.dd.put('pos',self.gvh.moat.state.pos,time.time(),pid)

            else:
                if self.dd.get('pos',pid+1) is None or self.dd.get('pos',pid-1) is None:
                    pass

                else:
                    point1 = self.dd.get('pos',pid+1)
                    point2 = self.dd.get('pos',pid-1)
                    target = midpoint(point1,point2)
                    self.gvh.moat.target = target

                self.dd.put('pos',self.gvh.moat.state.pos,time.time(),pid)
            print(n)
            if n >= 200:
                self.stop()

            try:
                self.__barrier.wait()
            except:
                continue


def midpoint(point1,point2):
    return point((point1.x + point2.x)/2,(point1.y+point2.y)/2,(point1.z+ point2.z)/2)
