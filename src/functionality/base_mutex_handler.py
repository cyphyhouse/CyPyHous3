# Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

import time
from typing import Union

from src.functionality.abstract.mutex_handler import MutexHandler
from src.objects.baseMutex import BaseMutex


class BaseMutexHandler(MutexHandler):

    def __init__(self, leader: bool, pid: int):
        super(BaseMutexHandler, self).__init__()
        self.__leader = leader
        self.__pid = pid
        self.__mutex_nums = {}
        self.__mutexes = {}

    @property
    def pid(self):
        return self.__pid

    @property
    def leader(self):
        return self.__leader

    @leader.setter
    def leader(self, leader: bool):
        self.__leader = leader

    @property
    def mutexnums(self):
        return self.__mutex_nums

    @property
    def mutexes(self):
        return self.__mutexes

    @property
    def pid(self):
        return self.__pid

    def add_mutex(self, mutex: BaseMutex) -> None:
        self.mutexes[mutex.mutex_id] = mutex
        self.mutexnums[mutex.mutex_id] = 0

    def add_request(self, index: str, pid: int, req_num: int):
        try:
            if (pid, req_num) not in self.mutexes[index].mutex_request_list:
                self.mutexes[index].mutex_request_list.append((pid,req_num))
        except KeyError:
            print("tried adding possibly undeclared mutex request")

    def grant_available_mutexes(self):
        if self.leader:
            for i in self.mutexes:
                if self.mutexes[i].mutex_holder is None:
                    self.mutexnums[i] += 1
                    self.mutexes[i].grant_mutex(self.mutexnums[i])

    def has_mutex(self, mutex_id):
        try:
            if self.mutexes[mutex_id].mutex_holder == self.pid:
                return True
        except KeyError:
            pass
        return False


    def run(self):
        while not self.stopped():
            self.grant_available_mutexes()
            time.sleep(0.1)
