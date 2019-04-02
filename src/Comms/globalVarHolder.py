#TODO:event listeners for UI input (?)
#TODO:check whether gvh needs to be iterable
import threading,time
from comms import comms
from commHandler import commHandler
class gvh(list):

    def __init__(self, name, participants = {}, dsm=None, sim=True, cs=None, ch=None):
        self.__name = name
        self.__sim = sim
        self.__participants = participants
        if sim :

            self.__cs = comms()
            self.__ch = commHandler()

        else:
            self.__cs = comms
            self.__ch = commHandler

        self.__dsm = dsm

    @property
    def name(self):
        return self.__name

    @property
    def comms(self):
        return self.__comms

    @property
    def commsHandler(self):
        return self.commsHandler

    @property
    def participants(self):
        return self.__participants

    @property
    def name(self):
        return self.__name

    @property
    def sim(self):
        return self.__sim

    @property
    def dsm(self):
        return self.__dsm

    @staticmethod
    def sleep(sl):
        try:
            time.sleep(sl)
        except IOError:
            print("error sleeping")


