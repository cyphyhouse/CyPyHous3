import time
from typing import Union

from src.functionality.mutex_handler import MutexHandler


class BaseMutexHandler(MutexHandler):

    def __init__(self, leader: bool, pid: int):
        super(BaseMutexHandler, self).__init__()
        self.leader = leader
        self.pid = pid
        self.mutex_nums = []
        self.__mutexes = []
        # self.start()


    @property
    def mutexes(self):
        return self.__mutexes

    def add_mutex(self, mutex):
        self.mutexes.append(mutex)
        self.mutex_nums.append(0)

    def add_request(self, index: int, pid: int, req_num: int):
        i = self.find_mutex_index(index)
        if i is not None:
            if (pid, req_num) not in self.mutexes[i].mutex_request_list:
                self.mutexes[i].mutex_request_list.append((pid, req_num))
        else:
            pass

    def find_mutex_index(self, mutex_id: int) -> Union[int, None]:
        for i in range(len(self.mutexes)):
            if self.mutexes[i].mutex_id == mutex_id:
                return i
        return None

    def grant_available_mutexes(self, mutex_nums: list):
        if self.leader:
            for i in range(len(self.mutexes)):
                # print(self.mutexes[i].mutex_request_list)
                if self.mutexes[i].mutex_holder is None:
                    # print("mutex available, granting")
                    # print(self.mutexnums[i])
                    self.mutex_nums[i] += 1
                    self.mutexes[i].grant_mutex(mutex_nums[i])

    def has_mutex(self, mutex_id):
        i = self.find_mutex_index(mutex_id)
        if i is not None:
            if self.mutexes[i].mutex_holder == self.pid:
                return True

        return False

    def run(self):
        while not self.stopped():
            self.grant_available_mutexes(self.mutex_nums)
            time.sleep(0.1)
