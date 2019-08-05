from abc import ABC, abstractmethod

from src.objects.message import Message


class Synchronizer(ABC):

    """
    abstract synchronizer object class. Can implement some form of round based
    synchronization algorithms.

    __is_synced : boolean whether is currently synchronized.
    """

    def __init__(self):
        """
        init method for abstract class synchronizer
        """
        self.__is_synced = False

    @property
    def is_synced(self) -> bool:
        """
        getter method for the is_synced field.
        :return: boolean whether synced.
        """
        return self.__is_synced

    @is_synced.setter
    def is_synced(self, sync: bool) -> None:
        """
        setter method for sync
        :param sync: bool sync
        :return: none
        """
        self.__is_synced = sync

    @abstractmethod
    def synchronize_wait(self):
        """
        any synchronizer object must implement a synchronization method
        """
        pass

    @abstractmethod
    def handle_sync_message(self, msg: Message):
        """
        any synchronizer object must implement a handle synchronize message method
        :return:
        """
