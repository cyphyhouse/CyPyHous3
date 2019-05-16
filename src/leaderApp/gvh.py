import threading,time
from comms import comms
from commHandler import commHandler
import moat
from positions import *

class gvh(list):

    def __init__(self, name, bcastqueue, participants = 1, sim=True):
        self.__name = name
        self.__sim = sim
        self.__participants = participants
        self.__comms = comms(bcastqueue)
        self.__commHandler = commHandler()
        self.__comms.start()
        self.__commHandler.start()
        import re
        global positions
        pid = int(re.sub(r'[a-zA-Z]', r'', name))
        self.__moat = moat.moat(pid,positions[pid])
        self.__moat.start()

    @property
    def name(self):
        return self.__name


    def pid(self):
        import re
        return int(re.sub(r'[a-zA-Z]', r'', self.name))



    def stop(self):
        self.moat.stop()




