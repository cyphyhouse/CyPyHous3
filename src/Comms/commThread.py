'''Abstract thread class to handle broadcast and receive messages for an agent'''

import threading
from abc import ABC, abstractmethod


class commThread(ABC, threading.Thread):

    def __init__(self):
        super(commThread, self).__init__()
        self._stop_event = threading.Event()

    def stop(self):
        self._stop_event.set()

    def stopped(self):
        return self._stop_event.is_set()

    @abstractmethod
    def setCommsHandler(self):
        pass

    @abstractmethod
    def write(self, message, destination=None):
        pass

    @abstractmethod
    def run(self):
        pass
