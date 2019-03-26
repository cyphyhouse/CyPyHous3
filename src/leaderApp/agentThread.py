from abc import ABC, abstractmethod
from threading import Thread, Event

from string import ascii_letters

class agentThread(ABC, Thread):

    def __init__(self, gvh,dd = None,locks = []):
        super(agentThread, self).__init__()
        self.__gvh = gvh
        #self.daemon = True
        self.numbots = gvh.participants
        self.name = gvh.name
        self.id = int(self.name.lstrip(ascii_letters))
        self._stop_event = Event()
        self._sleep_event = Event()
        self.dd = dd
        self.locks = locks
        self.start()

    @property
    def gvh(self):
        return self.__gvh

    def stop(self):
        self._stop_event.set()

    def stopped(self):
        return self._stop_event.is_set()

    def sleep(self):
        self._sleep_event.set()

    def is_sleeping(self):
        return self._sleep_event.is_set()

    @abstractmethod
    def run(self):
        pass
