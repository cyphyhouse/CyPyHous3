import time

from src.config.configs import AgentConfig
from src.functionality.comm_funcs import send
from src.functionality.synchronizer import Synchronizer
from src.harness.message_handler import round_update_create
from src.objects.message import Message


class BasicSynchronizer(Synchronizer):
    """
    basic synchronization method, where agents will wait for messages from all other agents to cross barrier.
    __participants : integer number of participants.
    __synclist : list of agents who have synced so far.
    __round_num : round number
    __ip_port_list : list of ports if running on same machine
    __retries : integer number of retries to synchronize
    """

    def __init__(self, a: AgentConfig):
        """
        init method for basic synchronizer object
        :param participants: number of participants to synchronize
        """
        super(BasicSynchronizer, self).__init__()
        self.pid = a.pid
        self.__participants = a.bots
        self.__sync_list = []
        self.__round_num = 0
        if not a.plist == []:
            self.__ip_port_list = a.plist
        else:
            self.__ip_port_list = [a.rport]
        self.__retries = 100

    @property
    def retries(self) -> int:
        return self.__retries

    @retries.setter
    def retries(self, retries: int):
        self.__retries = retries

    @property
    def ip_port_list(self):
        return self.__ip_port_list

    @property
    def sync_list(self) -> list:
        """
        getter method for the sync list
        :return:
        """
        return self.__sync_list

    @sync_list.setter
    def sync_list(self, sync_list: list) -> None:
        """
        setter method for sync_list
        :param sync_list: list to be set
        :return: none
        """
        self.__sync_list = sync_list

    @property
    def participants(self) -> int:
        """
        getter method for the number of participants
        :return:
        """
        return self.__participants

    @property
    def round_num(self) -> int:
        """
        getter method for round number
        :return: integer round number
        """
        return self.__round_num

    @round_num.setter
    def round_num(self, round_num: int) -> None:
        """
        setter method for round number
        :param round_num: int
        :return: none
        """
        self.__round_num = round_num

    def send_sync_message(self):
        for port in self.ip_port_list:
            send(round_update_create(self.pid, self.round_num, time.time()), AgentConfig.BROADCAST_ADDR, port)

    def synchronize_wait(self):
        # print("synchronizing", self.pid)
        tries = 0
        while not self.is_synced:
            tries += 1
            import time
            time.sleep(0.1)
            if tries == self.retries:
                print("giving up on sync")
                break
            if len(self.sync_list) == self.participants - 1:
                self.is_synced = True
                self.round_num += 1
                self.reset_synclist()
            else:
                continue
        self.is_synced = False

    def add_sync(self, agent: int) -> None:
        """
        add agent pid to sync list
        :param agent: agent
        :return:
        """
        self.__sync_list.append(agent)

    def reset_synclist(self) -> None:
        """
        reset synclist
        :return:
        """
        self.sync_list = []

    def handle_sync_message(self, msg: Message):
        """
        synchronization message handling
        :param msg: message to be handled.
        :return:
        """
        print("here")
        sender = msg.sender
        round_num = msg.content
        if round_num > self.round_num + 1:
            print("error")
        else:
            if sender in self.sync_list or sender == self.pid:
                pass
            else:
                self.add_sync(sender)


class RoundSyncError(Exception):
    "raised when there is an exception in synchronization"
    pass
