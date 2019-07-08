import time
from threading import Thread, Event
from typing import Union


class BaseMutexHandler(Thread):

    def __init__(self, leader: bool, pid: int):
        super(BaseMutexHandler, self).__init__()
        self.leader = leader
        self.pid = pid
        self.mutexnums = []
        self.__mutexes = []
        self.__stop_event = Event()
        self.start()

    def stop(self) -> None:
        """
         a flag to set to to safely exit the thread
        :return: None
        """
        self.__stop_event.set()

    def stopped(self) -> bool:
        """
        set the stop flag
        :return: true if stop event is set, false otherwise
        """
        return self.__stop_event.is_set()

    @property
    def mutexes(self):
        return self.__mutexes

    def add_mutex(self, mutex):
        self.mutexes.append(mutex)
        self.mutexnums.append(0)

    def add_request(self, index: int, pid: int, req_num:int):
        print("adding request from",pid)
        i = self.find_mutex_index(index)
        if i is not None:
            if (pid, req_num) not in self.mutexes[i].mutex_request_list:
                self.mutexes[i].mutex_request_list.append((pid,req_num))
        else:
            pass

    def find_mutex_index(self, mutex_id: int) -> Union[int, None]:
        for i in range(len(self.mutexes)):
            if self.mutexes[i].mutex_id == mutex_id:
                return i
        return None

    def grant_available_mutexes(self, mutexnums:list):
        if self.leader:
            for i in range(len(self.mutexes)):
                #print(self.mutexes[i].mutex_request_list)
                if self.mutexes[i].mutex_holder is None:
                    #print("mutex available, granting")
                    #print(self.mutexnums[i])
                    self.mutexnums[i] += 1
                    self.mutexes[i].grant_mutex(mutexnums[i])

    def has_mutex(self, mutex_id):
        i = self.find_mutex_index(mutex_id)
        if i is not None:
            if self.mutexes[i].mutex_holder == self.pid:
                return True

        return False


    def run(self):
        while not self.stopped():
            self.grant_available_mutexes(self.mutexnums)
            time.sleep(0.1)
