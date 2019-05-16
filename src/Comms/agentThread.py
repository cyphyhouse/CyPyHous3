from threading import Thread, Event
from abc import ABC, abstractmethod

class agentThread(ABC, Thread):

    def __init__(self,gvh,configfile):
        super(agentThread, self).__init__()
        self._stop_event = Event()
        self.gvh = gvh
        self.configfile = configfile
        self.start()


    def stop(self):
        self._stop_event.set()

    def stopped(self):
        return self._stop_event.is_set()

    @abstractmethod
    def run(self):
        pass

