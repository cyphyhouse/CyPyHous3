# Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

import typing as tp

from src.datatypes.robot import BotType
from src.motion.abstract.planner import Planner


class AgentConfig(object):
    """
    agent configuration object
    __pid : unique integer identifier of the agent
    __rip : receiver ip
    __rport : receiver (listener) port
    __plist : list of ports (used when multiple instances of koord program running on same robot)
    __mutex_handler : mutual exclusion handler object
    __is_leader : property determining whether the agent is leader (used in conflict resolution in various cases
    __moat_class : motion automaton specification
    """

    def __init__(self, pid: int, bots: int, rip: str, rport: int, plist: tp.Union[tp.List[int], None] = None,
                 mh: tp.Union[None, classmethod] = None, is_leader=False,
                 moat_class: tp.Union[None, classmethod] = None,
                 mhargs: tp.Union[tp.List, None] = None):
        """

        :param pid : unique integer identifier
        :type pid : int
        :param bots : number of robots
        :type bots : int
        :param rip : receiver ip
        :type ip : str
        :param rport : listener port
        :type rport : int
        :param plist : list of listener ports
        :type plist : list(int)
        :param mh : mutual handler
        :type mh : classmethod, None
        :param is_leader : boolean indicating whether robot is leader
        :type is_leader : bool
        :param moat_class : motion automaton class
        :type moat_class : classmethod, None
        :param mhargs : mutual handler arguments
        :type mhargs : list
        """
        if plist is None:
            plist = []
        if mhargs is None:
            mhargs = []
        if mh is not None:
            self.__mutex_handler = mh(*mhargs)
        else:
            self.__mutex_handler = None
        self.__pid = pid
        self.__bots = bots
        self.__rip = rip
        self.__rport = rport
        self.__plist = plist
        self.__is_leader = is_leader
        self.__moat_class = moat_class

    # ------------ MEMBER ACCESS METHODS --------------

    @property
    def pid(self) -> int:
        """
        :return: pid
        :rtype: int
        """
        return self.__pid

    @property
    def bots(self) -> int:
        """
        :return: number of bots
        :rtype: int
        """
        return self.__bots

    @property
    def rip(self) -> str:
        """
        :return: receiver ip
        :rtype: str
        """
        return self.__rip

    @property
    def rport(self) -> int:
        """
        :return: listener port
        :rtype: int
        """
        return self.__rport

    @property
    def plist(self) -> tp.List[int]:
        """
        :return: list of ports
        :rtype: List[int]
        """
        return self.__plist

    @property
    def is_leader(self) -> bool:
        """
        :return: true if leader false otherwise
        :rtype: bool
        """
        return self.__is_leader

    @property
    def moat_class(self) -> tp.Union[None, classmethod]:
        """
        :return: motion automaton object (if any)
        :rtype: Union[None, classmethod]
        """
        return self.__moat_class

    @property
    def mutex_handler(self) -> tp.Union[None, classmethod]:
        """
        :return: mutex handler (if any)
        :rtype: Union[None,classmethod]
        """
        return self.__mutex_handler

    @pid.setter
    def pid(self, pid: int) -> None:
        """
        :param: pid
        :type: int
        """
        self.__pid = pid

    @bots.setter
    def bots(self, bots: int) -> None:
        """
        :param: number of bots
        :type: int
        """
        self.__bots = bots

    @rip.setter
    def rip(self, rip: str) -> None:
        """
        :param: receiver ip
        :type: str
        """
        self.__rip = rip

    @rport.setter
    def rport(self, rport: int) -> None:
        """
        :param: listener port
        :type: int
        """
        self.__rport = rport

    @plist.setter
    def plist(self, plist: tp.List[int]) -> None:
        """
        :param: list of ports
        :type: List[int]
        """
        self.__plist = plist

    @is_leader.setter
    def is_leader(self, is_leader: bool) -> None:
        """
        :param: true if leader false otherwise
        :type: bool
        """
        self.__is_leader = is_leader

    @moat_class.setter
    def moat_class(self, moat_class: tp.Union[None, classmethod]) -> None:
        """
        :param: motion automaton object (if any)
        :type: Union[None, classmethod]
        """
        self.__moat_class = moat_class

    @mutex_handler.setter
    def mutex_handler(self, mutex_handler: tp.Union[None, classmethod]) -> None:
        """
        :param: mutex handler (if any)
        :type: Union[None,classmethod]
        """
        self.__mutex_handler = mutex_handler

    def __repr__(self):
        """
        string representation
        """
        s = ""
        s += "pid" + ":" + str(self.pid) + "\n"
        s += "bots" + ":" + str(self.bots) + "\n"
        s += "rip" + ":" + str(self.rip) + "\n"
        s += "rport" + ":" + str(self.rport) + "\n"
        s += "plist" + ":" + str(self.plist) + "\n"
        s += "mutex_handler" + ":" + str(self.mutex_handler) + "\n"
        s += "is_leader" + ":" + str(self.is_leader) + "\n"
        s += "moat_class" + ":" + str(self.moat_class) + "\n"
        return s

    # ----------------- OPERATIONS --------------------

    def __eq__(self, other):
        """
        checking equality
        """
        return self.pid == other.pid and \
               self.bots == other.bots and \
               self.rip == other.rip and \
               self.rport == other.rport and \
               self.plist == other.plist and \
               self.mutex_handler == other.mutex_handler and \
               self.is_leader == other.is_leader and \
               self.moat_class == other.moat_class


class MoatConfig(object):
    """
    motion configuration object
    __waypoint_topic : msg type to publish to set waypoints for controller
    __reached_topic : reached or not
    __rospy_node : rospy node corresponding to motion automaton
    __bot_name : unique string name for robot
    __queue_size : number of ros messages stored
    __bot_type : bot type (to determine planner , etc)
    __pos_node : position node (ros)
    __pos_msg_type : position message type (ros)
    __rchd_msg_type : reached message type (ros)
    __planner : path planner
    """

    def __init__(self, waypoint_topic: str, reached_topic: str, rospy_node: str, bot_name: str, queue_size: int,
                 bot_type: BotType, pos_node: str, pos_msg_type=None, rchd_msg_type=None,
                 planner: tp.Optional[Planner] = None):
        """
        :param waypoint_topic: waypoint message rostopic
        :type waypoint_topic: str

        :param reached_topic: reached message rostopic
        :type reached_topic: str

        :param rospy_node: rospy node name
        :type rospy_node: str

        :param bot_name: unique string bot name
        :type bot_name: str

        :param queue_size: queue size for ros messages
        :type queue_size: int

        :param bot_type: bot type
        :type bot_type: BotType

        :param pos_node: position node name (ros)
        :type pos_node: str

        :param pos_msg_type: position message type
        :type pos_msg_type: (determined by ros message)

        :param rchd_msg_type: reached message type
        :type rchd_msg_type: (determined by ros message)

        :param planner: path planner
        :type planner: Planner
        """
        self.__waypoint_topic = waypoint_topic
        self.__reached_topic = reached_topic
        self.__pos_node = pos_node
        self.__pos_msg_type = pos_msg_type
        self.__rchd_msg_type = rchd_msg_type
        self.__rospy_node = rospy_node
        self.__bot_name = bot_name
        self.__queue_size = queue_size
        self.__bot_type = bot_type
        self.__planner = planner

    # ------------ MEMBER ACCESS METHODS --------------

    @property
    def waypoint_topic(self) -> str:
        """
        :return: waypoint ros topic
        :rtype: str
        """
        return self.__waypoint_topic

    @waypoint_topic.setter
    def waypoint_topic(self, waypoint_topic: str) -> None:
        """
        :param waypoint_topic: waypoint ros topic
        :type: waypoint_topic: str
        """
        self.__waypoint_topic = waypoint_topic

    @property
    def reached_topic(self) -> str:
        """
        :return: reached ros topic
        :rtype: str
        """
        return self.__reached_topic

    @reached_topic.setter
    def reached_topic(self, reached_topic: str) -> None:
        """
        :param reached_topic: reached ros topic
        :type: reached_topic: str
        """
        self.__reached_topic = reached_topic

    @property
    def pos_node(self) -> str:
        """
        :return: position node name
        :rtype: str
        """
        return self.__pos_node

    @pos_node.setter
    def pos_node(self, pos_node: str) -> None:
        """
        :param pos_node: position node
        :type pos_node: str
        """
        self.__pos_node = pos_node

    @property
    def pos_msg_type(self):
        """
        :return: pos message type
        """
        return self.__pos_msg_type

    @pos_msg_type.setter
    def pos_msg_type(self, pos_msg_type) -> None:
        """
        :param pos_msg_type: position message type
        """
        self.__pos_msg_type = pos_msg_type

    @property
    def rchd_msg_type(self):
        """
        :return: reached message type
        """
        return self.__rchd_msg_type

    @rchd_msg_type.setter
    def rchd_msg_type(self, rchd_msg_type: str) -> None:
        """
        :param rchd_msg_type: reached message type
        """
        self.__rchd_msg_type = rchd_msg_type

    @property
    def rospy_node(self) -> str:
        """
        :return: rospy node
        :rtype: str
        """
        return self.__rospy_node

    @rospy_node.setter
    def rospy_node(self, rospy_node: str) -> None:
        """
        :param rospy_node: name of the rospy node
        :type: str
        """
        self.__rospy_node = rospy_node

    @property
    def queue_size(self) -> int:
        """
        :return: queue size of ros messages
        :rtype: int
        """
        return self.__queue_size

    @queue_size.setter
    def queue_size(self, queue_size: int) -> None:
        """
        :param queue_size: queue size setter
        :type: int
        """
        self.__queue_size = queue_size

    @property
    def bot_name(self) -> str:
        """
        :return: bot name
        :rtype: str
        """
        return self.__bot_name

    @bot_name.setter
    def bot_name(self, bot_name: str) -> None:
        """
        :param bot_name: bot name
        :type: str
        """
        self.__bot_name = bot_name

    @property
    def bot_type(self):
        """
        :return: bot type
        :rtype: BotType
        """
        return self.__bot_type

    @bot_type.setter
    def bot_type(self, bot_type: BotType) -> None:
        """
        :param bot_type: bot type
        :type: BotType
        """
        self.__bot_type = bot_type

    @property
    def planner(self) -> Planner:
        """
        :return: path planner
        :rtype: Planner
        """
        return self.__planner

    @planner.setter
    def planner(self, planner: Planner) -> None:
        """
        :param planner: path planner
        :type: Planner
        """
        self.__planner = planner

    def __repr__(self):
        s = ""
        s += "waypoint_topic" + ":" + str(self.waypoint_topic) + "\n"
        s += "reached_topic" + ":" + str(self.reached_topic) + "\n"
        s += "pos_msg_type" + ":" + str(self.pos_msg_type) + "\n"
        s += "rchd_msg_type" + ":" + str(self.rchd_msg_type) + "\n"
        s += "rospy_node" + ":" + str(self.rospy_node) + "\n"
        s += "pos_node" + ":" + str(self.pos_node) + "\n"
        s += "bot_name" + ":" + str(self.bot_name) + "\n"
        s += "queue_size" + ":" + str(self.queue_size) + "\n"
        s += "bot_type" + ":" + str(self.bot_type) + "\n"
        s += "planner" + ":" + str(self.planner)
        return s


def gen_positioning_params(config: MoatConfig) -> tp.Tuple:
    """
    TODO: write unit test
    function to generate positioning message parameters from the motion configuration
    :param config: Motion automaton Config
    :type config: MoatConfig
    :return: positioning parameters
    :rtype: Tuple
    """
    return config.pos_node + config.bot_name + '/pose', config.pos_msg_type


def gen_reached_params(config: MoatConfig) -> tp.Tuple:
    """
    TODO: write unit test
    generate reached message parameters from the motion configuration
    :param config: Motion automaton Config
    :type config: MoatConfig
    :return: positioning parameters
    :rtype: Tuple
    """
    # return config.bot_name + '/' + config.rospy_node + '/' + config.reached_topic, config.rchd_msg_type
    return config.rospy_node + '/' + config.reached_topic, config.rchd_msg_type


def gen_waypoint_params(config: MoatConfig):
    """
    TODO: write unit test
    generate waypoint message parameters from motion configuration
    :param config: Motion automaton Config
    :type config: MoatConfig
    :return: waypoint parameters
    :rtype: Tuple
    """
    # return config.bot_name + '/' + config.rospy_node + '/' + config.waypoint_topic, config.pos_msg_type
    return config.rospy_node + '/' + config.waypoint_topic, config.pos_msg_type
