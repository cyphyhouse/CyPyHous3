#TODO: implement a safe-ish "stop"


import threading
from abc import ABC,abstractmethod


class agentThread(ABC,threading.Thread):

    def __init__(self,name=None):
        super(agentThread,self).__init__()
        self._stop_event = threading.Event()
        if name is None:
            self.name = super().name
        else:
            self.name = name
        #self.daemon = True
        self.start()

    def stop(self):
        self._stop_event.set()

    def stopped(self):
        return self._stop_event.is_set()

    @abstractmethod
    def run(self):
        pass
