from abc import ABC, abstractmethod


class Mutex(ABC):
    """
    abstract mutual exclusion class
    """

    def __init__(self):
        """
        base init method
        """
        pass

    @abstractmethod
    def request_mutex(self, *args, **kwargs):
        """
        request mutex abstract method
        :return:
        """
        pass

    @abstractmethod
    def grant_mutex(self, *args, **kwargs):
        """
        grant mutex abstract method
        :return:
        """
        pass

    @abstractmethod
    def release_mutex(self, *args, **kwargs):
        """
        release mutex abstract method
        :return:
        """
        pass
