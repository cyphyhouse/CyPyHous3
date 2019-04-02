'''Abstract thread class to handle broadcast and receive messages for an agent'''

from threading import Thread, Event
from abc import ABC, abstractmethod
broadcast = '192.168.1.255'

class commThread(ABC, Thread):
    def __init__(self):
        super(commThread, self).__init__()
        self.__msglist = []
        try:
            self.__gvh = gvh
            self.__name = gvh.name
        except:
            print("Error in commthread.__init__, perhaps gvh is None")
        self.__ip = broadcast
        self._stop_event = Event()

    @property
    def msglist(self):
        return self.__msglist

    @property
    def channel(self):
        return self.__channel

    @property
    def name(self):
        return self.__name

    @property
    def ip(self):
        return self.__ip

    @property
    def gvh(self):
        return self.__gvh

    def stop(self):
        self._stop_event.set()

    def stopped(self):
        return self._stop_event.is_set()

    @abstractmethod
    def write(self, message, destination=None):
        pass

    @abstractmethod
    def run(self):
        pass
