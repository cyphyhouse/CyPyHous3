import threading,time
from comms import comms
from commHandler import commHandler
import moat

class gvh(list):

    def __init__(self, name, bcastqueue, participants = 1, sim=True):
        self.__name = name
        self.__sim = sim
        self.__participants = participants
        self.__comms = comms(bcastqueue)
        self.__commHandler = commHandler()
        self.__comms.start()
        self.__commHandler.start()
        self.__moat = moat.moat()
        self.__moat.start()

    @property
    def moat(self):
        return self.__moat

    @property
    def comms(self):
        return self.__comms

    @property
    def commHandler(self):
        return self.__commHandler

    @property
    def name(self):
        return self.__name

    def pid(self):
        import re
        return int(re.sub(r'[a-zA-Z]', r'', self.name))


    @property
    def participants(self):
        return self.__participants

    @property
    def sim(self):
        return self.__sim

    def stop(self):
        self.__moat.stop()




