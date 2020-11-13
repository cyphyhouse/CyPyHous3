import abc
from threading import Event, Thread
from typing import Optional

from src.config.configs import AgentConfig, MoatConfig
from src.datatypes.time_utils import Rate


class AgentThread(Thread, abc.ABC):
    def __init__(self, agent_config: AgentConfig,
                 moat_config: Optional[MoatConfig] = None) -> None:
        super(AgentThread, self).__init__()

        self.__stop_event = Event()
        self.__delta = 1.0  # second TODO read from agent_config

        if moat_config is None:
            self.__moat = None  # type: Optional[MotionAutomaton]
        else:
            self.__moat = agent_config.moat_class(moat_config)

    @property
    def delta(self) -> float:
        return self.__delta

    def stop(self) -> None:
        self.__stop_event.set()

    @abc.abstractmethod
    def _initialize_vars(self) -> None:
        """ abstract method to initialize variables
        :return:
        """
        raise NotImplementedError

    @abc.abstractmethod
    def _loop_body(self) -> None:
        """ loop body
        :return:
        """
        raise NotImplementedError

    def run(self) -> None:
        self._initialize_vars()

        rate = Rate(1 / self.__delta)
        while not self.__stop_event.is_set():
            # Put sleep in the loop head to ensure the rate with all branches and continue statements
            rate.sleep()

            self._loop_body()
