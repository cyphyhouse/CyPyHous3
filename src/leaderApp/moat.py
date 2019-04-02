import math
import time
import moat_state
from threading import Thread,Event
import sys


#sys.setrecursionlimit(1000000)

class moat(Thread):
    def __init__(self,rd = 5):
        super(moat,self).__init__()
        self.__state = moat_state.moat_state()
        self.__reacheddist = rd
        self.__target = None
        self._stop_event = Event()
        self.__inmotion = False

    def stop(self):
        self._stop_event.set()

    def stopped(self):
        return self._stop_event.is_set()

    @property
    def target(self):
        return self.__target

    @target.setter
    def target(self,pt):
        self.target(pt)


    @property
    def inmotion(self):
        return self.__inmotion

    @property
    def state(self):
        return self.__state

    def goto(self,position):
        if self.state.position[0] != position[0]:
            angleto = math.atan((position[1] - self.state.position[1])/(position[0] - self.state.position[0]))
            self.state.xacc = float(2) * math.cos(angleto)
            self.state.yacc = float(2) * math.sin(angleto)
            self.inmotion = True

    def distanceTo(self,point):
        return math.sqrt((self.state.position[0] - point[0])**2 + (self.state.position[1] - point[1])**2)

    def reached(self,point):
        if  self.distanceTo(point) <= self.reacheddist:
            return True
        return False

    def run(self):
        while (not self.stopped()):
            time.sleep(1)
            if self.target is not None:
                while self.inmotion:
                    time.sleep(1)
                    self.goto(self.target)
                    self.state.update()
                    if self.reached(self.target):
                        self.inmotion = False
                    else:
                        break




    @state.setter
    def state(self, value):
        self._state = value

    @inmotion.setter
    def inmotion(self, value):
        self._inmotion = value



