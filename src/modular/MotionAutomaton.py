import math
import time
from threading import Thread,Event
from point import *
from state import *


class moat(Thread):
    def __init__(self, pid, rd = 2,configfile = 'default'):

        super(moat,self).__init__()
        self.__pid = pid
        self.__state = state(pid,configfile)
        self.__rdist = rd
        self.__target = None
        self._stop_event = Event()
        self.__inmotion = False

    def stop(self):
        self._stop_event.set()

    def stopped(self):
        return self._stop_event.is_set()

    @property
    def pid(self):
        """
        getter method for pid
        :return:
        """
        return self.__pid

    @pid.setter
    def pid(self,pid):
        """
        setter method for pid
        :param pid:
        :return:
        """
        self.__pid = pid

    @property
    def state(self):
        """
        getter method for state
        :return:
        """
        return self.__state

    @state.setter
    def state(self, state):
        """
        setter method for state
        :param pid:
        :return:
        """
        self.__state = state

    @property
    def rdist(self):
        return self.__rdist

    @rdist.setter
    def rdist(self,rdist):
        self.__rdist = rdist

    @property
    def target(self):
        return self.__target

    @target.setter
    def target(self,target):
        self.__target = target

    @property
    def inmotion(self):
        return self.__inmotion

    @inmotion.setter
    def inmotion(self,inmotion):
        self.__inmotion = inmotion

    def goto(self,pos):
        speed = self.state.speed
        xcomp = speed * math.cos(self.angleto(pos))
        ycomp = speed * math.sin(self.angleto(pos))
        if (pos.x - self.state.pos.x) * xcomp > 0 :
            self.state.__xvel = xcomp
        else:
            self.state.__xvel = -1 * xcomp

        if (pos.y - self.state.pos.y) * ycomp > 0 :
            self.state.__yvel = ycomp
        else:
            self.state.__yvel = -1 * ycomp



    def distanceTo(self,point1):
        return distancebetween(point1,self.state.pos)

    def angleto(self,point1):
        return slope(self.state.pos,point1)

    def reached(self,point):
        if point is None:
            return True
        if  distancebetween(self.state.pos,point) <= self.rdist:
            return True
        else:
            return False

    def run(self):
        while not self.stopped():
            time.sleep(1)
            if self.target is not None:
                self.__inmotion = True

            while self.inmotion:
                time.sleep(1)
                self.goto(self.target)
                self.state.update()
                if self.reached(self.target):
                    self.__inmotion = False
                    self.__target = None







