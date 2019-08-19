import time
from typing import Union

from src.functionality.mutex_handler import MutexHandler


class BaseMutexHandler(MutexHandler):

    def __init__(self, leader: bool, pid: int):
        super(BaseMutexHandler, self).__init__()
        # test comment
        self.__leader = leader
        self.__pid = pid
        self.__mutex_nums = []
        self.__mutexes = []
        # self.start()

    @property
    def pid(self):
        return self.__pid

    @property
    def mutexnums(self):
        return self.__mutex_nums

    @property
    def mutexes(self):
        return self.__mutexes

    @property
    def mutexes(self):
        return self.__mutexes

    def add_mutex(self, mutex):
        self.__mutexes.append(mutex)
        self.__mutex_nums.append(0)

    def add_request(self, index: int, pid: int, req_num: int):
        i = self.find_mutex_index(index)
        if i is not None:
            if (pid, req_num) not in self.__mutexes[i].mutex_request_list:
                print("request from",pid,"added")
                self.__mutexes[i].mutex_request_list.append((pid, req_num))
            if self.__leader:
                pass
                # self.__mutexes[i].ack_request(pid, req_num)
        else:
            pass

    def find_mutex_index(self, mutex_id: int) -> Union[int, None]:
        for i in range(len(self.__mutexes)):
            if self.__mutexes[i].mutex_id == mutex_id:
                return i
        return None

    def grant_available_mutexes(self, mutex_nums: list):
        if self.__leader:
            for i in range(len(self.__mutexes)):
                #print("mutex",self.__mutexes[i].mutex_id,"holder is",self.__mutexes[i].mutex_holder)
                if self.__mutexes[i].mutex_holder is None:
                    self.__mutex_nums[i] += 1
                    self.__mutexes[i].grant_mutex(mutex_nums[i])

    def has_mutex(self, mutex_id):
        i = self.find_mutex_index(mutex_id)
        if i is not None:
            if self.__mutexes[i].mutex_holder == self.__pid:
                return True

        return False

    def run(self):
        while not self.stopped():
            self.grant_available_mutexes(self.__mutex_nums)
            time.sleep(0.1)
