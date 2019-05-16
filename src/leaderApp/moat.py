import math
import time
import moat_state
import point
from threading import Thread,Event
import sys
from targets import *

#sys.setrecursionlimit(1000000)

class moat(Thread):
    def __init__(self,pid,position,rd = 2):

        super(moat,self).__init__()
        self.pid = pid
        self.state = moat_state.moat_state(position)
        self.reacheddist = rd
        self.target = None
        self._stop_event = Event()
        self.inmotion = False

    def stop(self):
        self._stop_event.set()

    def stopped(self):
        return self._stop_event.is_set()


    def set_target(self, pt):
        self.target = pt

    def goto(self,position):
        if not float(self.state.position.x) == float(position.x):

            angleto = math.atan((position.y - self.state.position.y)/(position.x - self.state.position.x))
            self.state.set_xvel(float(2) * math.cos(angleto))
            self.state.set_yvel(float(2) * math.sin(angleto))
            self.inmotion = True
        else:
            self.state.set_xvel(0)
            self.state.xacc = 0
            if float(self.state.position.y) > float(position.y):
                self.state.yvel = -2
            else:
                self.state.yvel = 2



    def distanceTo(self,point):
        return math.sqrt((self.state.position.x - point.x)**2 + (self.state.position.y - point.y)**2)

    def reached(self,point):
        if point is None:
            return False
        if  self.distanceTo(point) <= self.reacheddist:

            return True
        else:
            return False

    def run(self):
        updatefile = "positions"+str(self.pid)
        f = open(updatefile,'w')
        while (not self.stopped()):
            time.sleep(1)
            self.target = targets[self.pid]
            if self.target is not None:
                self.inmotion = True
            else:
                pass
            while self.inmotion:
                time.sleep(0.3)
                self.goto(self.target)
                self.state.update()
                f.write(str(self.state.position.x)+","+str(self.state.position.y)+"\n")
                if self.reached(self.target):
                    self.inmotion = False
                    self.target = None
                    targets[self.pid] = self.target
                else:
                    pass






