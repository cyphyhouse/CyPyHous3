from abc import ABC, abstractmethod
from threading import Thread, Event


class AgentThread(ABC, Thread):

    def __init__(self, gvh, dd=None, locks=[],index = 0):
        """
        abstract object for each agent thread.
        :param gvh: the global variable holder. contains info about all other robots, motion automata, comms, etc
        :param dd: distributed shared memory
        :param locks: if there are any locks on the shared variables, this stores them.
        """
        super(AgentThread, self).__init__()
        self.gvh = gvh
        self.__numbots = gvh.participants
        self.__name = gvh.name
        self.__id = gvh.pid
        self._stop_event = Event()
        self._sleep_event = Event()
        self.__dd = dd
        self.__locks = locks
        self.start()



    @property
    def numbots(self):
        """
        getter method for number of robots
        :return: numbots
        """
        return self.__numbots

    @numbots.setter
    def numbots(self, numbots):
        """
        setter method for numbots
        :param gvh: gvh
        :return:
        """
        self.__numbots = numbots

    @property
    def name(self):
        """
        getter method for name
        :return:
        """
        return self.__name

    @name.setter
    def name(self, name):
        """
        setter method for name
        :param name: name
        :return:
        """
        self.__name = name

    @property
    def id(self):
        """
        getter method for the unique numerical identifier
        :return:
        """
        return self.__id

    @id.setter
    def id(self, id):
        """
        setter method for id
        :param id: id
        :return:
        """
        self.__id = id

    @property
    def dd(self):
        """
        getter method for the distributed shared memory object
        :return:
        """
        return self.__dd

    @dd.setter
    def dd(self, dd):
        """
        setter method for dd
        :param gvh: dd
        :return:
        """
        self.__dd = dd

    @property
    def locks(self):
        """
        getter method for the locks
        :return:
        """
        return self.__locks

    @locks.setter
    def locks(self, locks):
        """
        setter method for locks
        :param gvh: locks
        :return:
        """
        self.__locks = locks

    def stop(self):
        """
         a flag to set to to safely exit the thread
        :return:
        """
        self._stop_event.set()
        self.gvh.moat.stop()

    def stopped(self):
        """
        set the stop flag
        :return:
        """
        return self._stop_event.is_set()

    def sleep(self):
        """
        allows sleeping
        :return:
        """
        self._sleep_event.set()

    def is_sleeping(self):
        """
        sets the sleeping flag
        :return:
        """
        return self._sleep_event.is_set()

    @abstractmethod
    def run(self):
        """
        needs to be implemented for any agenThread
        :return:
        """
        pass
