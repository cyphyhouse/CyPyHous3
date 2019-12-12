# Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

import time
from typing import Dict

from src.functionality.abstract.mutexHandler import MutexHandler
from src.objects.basicMutex import BasicMutex


class BasicMutexHandler(MutexHandler):
    """
    mutex handler for Base Mutexes

    __leader : whether current agent is the leader
    __pid : unique integer pid of agent
    __mutex_nums : dictionary storing the request number for each mutex
    __mutexes : dictionary of available mutexes
    """

    def __init__(self, leader: bool, pid: int):
        super(BasicMutexHandler, self).__init__()
        self.__leader = leader
        self.__pid = pid
        self.__mutex_nums = {}
        self.__mutexes = {}

    # ------------ MEMBER ACCESS METHODS --------------

    @property
    def pid(self) -> int:
        return self.__pid

    @property
    def leader(self) -> bool:
        return self.__leader

    @property
    def mutex_nums(self) -> Dict:
        return self.__mutex_nums

    @property
    def mutexes(self) -> Dict:
        return self.__mutexes

    @leader.setter
    def leader(self, leader: bool) -> None:
        self.__leader = leader

    @mutexes.setter
    def mutexes(self, mutexes: Dict) -> None:
        self.__mutexes = mutexes

    @mutex_nums.setter
    def mutex_nums(self, mutex_nums: Dict) -> None:
        self.__mutex_nums = mutex_nums

    @pid.setter
    def pid(self, pid: int):
        self.__pid = pid

    # ------------ MUTEX MANAGEMENT METHODS --------------

    def add_mutex(self, mutex: BasicMutex) -> None:
        """
        add a mutex

        :param mutex: Mutex to be added
        :type mutex: BasicMutex

        """
        self.__mutexes[mutex.mutex_id] = mutex
        self.__mutex_nums[mutex.mutex_id] = 0

    def add_request(self, key: str, pid: int, req_num: int):
        """
        add a mutex request

        :param key: name of mutex
        :type key: str

        :param pid: pid of requesting agent
        :type pid: int

        :param req_num: request number to differentiate between multiple requests
        :type req_num: int

        """
        try:
            if (pid, req_num) not in self.__mutexes[key].mutex_request_list:
                self.__mutexes[key].mutex_request_list.append((pid, req_num))
        except KeyError:
            print("tried adding possibly undeclared mutex request")

    def grant_available_mutexes(self) -> None:
        """
        method to grant all available mutexes
        """
        if self.__leader:
            for i in self.__mutexes:
                if self.__mutexes[i].mutex_holder is None:
                    self.__mutex_nums[i] += 1
                    self.__mutexes[i].grant_mutex(self.mutex_nums[i])

    def has_mutex(self, mutex_id: str) -> bool:
        """
        checks whether current agent has the mutex b

        :param mutex_id: name of the mutex being checked
        :type mutex_id: str
        """
        try:
            if self.__mutexes[mutex_id].mutex_holder == self.__pid:
                return True
        except KeyError:
            pass
        return False

    def run(self):
        """
        keep granting available mutexes
        """
        while not self.stopped():
            self.grant_available_mutexes()
            time.sleep(0.1)
