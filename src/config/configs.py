# Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

import typing as tp

from src.datatypes.robot import BotType
from src.motion.abstract.planner import Planner
from src.motion.simpleplanner import SimplePlanner


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
    __send_ips: IPs with corresponding ports of other participating systems (default is broadcast)
    """

    def __init__(self, pid: int, bots: int, rip: str, rport: int, plist: tp.Union[tp.List[int], None] = None,
                 mh: tp.Union[None, classmethod] = None, is_leader=False,
                 moat_class: tp.Union[None, classmethod] = None,
                 mhargs: tp.Union[tp.List, None] = None, send_ips: tp.List[tp.Tuple[str, int]] = None):
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

        :param send_ips : list of ips participating in the application, with corresponding ports
        :type send_ips : List[Tuple[str,int]]
        """
        if plist is None:
            plist = []
        if mhargs is None:
            mhargs = []
        if mh is not None:
            self.__mutex_handler = mh(*mhargs)
        else:
            self.__mutex_handler = None
        if send_ips is None:
            send_ips = []
        self.__pid = pid
        self.__bots = bots
        self.__rip = rip
        self.__rport = rport
        self.__plist = plist
        self.__is_leader = is_leader
        self.__moat_class = moat_class
        self.__send_ips = send_ips

    # ------------ MEMBER ACCESS METHODS --------------

    @property
    def pid(self) -> int:
        return self.__pid

    @property
    def bots(self) -> int:
        return self.__bots

    @property
    def rip(self) -> str:
        return self.__rip

    @property
    def rport(self) -> int:
        return self.__rport

    @property
    def plist(self) -> tp.List[int]:
        return self.__plist

    @property
    def is_leader(self) -> bool:
        return self.__is_leader

    @property
    def moat_class(self) -> tp.Union[None, classmethod]:
        return self.__moat_class

    @property
    def mutex_handler(self) -> tp.Union[None, classmethod]:
        return self.__mutex_handler

    @pid.setter
    def pid(self, pid: int) -> None:
        self.__pid = pid

    @bots.setter
    def bots(self, bots: int) -> None:
        self.__bots = bots

    @rip.setter
    def rip(self, rip: str) -> None:
        self.__rip = rip

    @rport.setter
    def rport(self, rport: int) -> None:
        self.__rport = rport

    @plist.setter
    def plist(self, plist: tp.List[int]) -> None:
        self.__plist = plist

    @is_leader.setter
    def is_leader(self, is_leader: bool) -> None:
        self.__is_leader = is_leader

    @moat_class.setter
    def moat_class(self, moat_class: tp.Union[None, classmethod]) -> None:
        self.__moat_class = moat_class

    @mutex_handler.setter
    def mutex_handler(self, mutex_handler: tp.Union[None, classmethod]) -> None:
        self.__mutex_handler = mutex_handler

    @property
    def send_ips(self) -> tp.List[tp.Tuple[str, int]]:
        return self.__send_ips

    @send_ips.setter
    def send_ips(self, send_ips: tp.List[tp.Tuple[str, int]]):
        self.__send_ips = send_ips

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
        s += "ip_port_list" + ":" + str(self.send_ips)
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
                 planner: Planner = SimplePlanner()):
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
        return self.__waypoint_topic

    @property
    def reached_topic(self) -> str:
        return self.__reached_topic

    @property
    def pos_node(self) -> str:
        return self.__pos_node

    @property
    def pos_msg_type(self):
        return self.__pos_msg_type

    @property
    def rchd_msg_type(self):
        return self.__rchd_msg_type

    @property
    def rospy_node(self) -> str:
        return self.__rospy_node

    @property
    def queue_size(self) -> int:
        return self.__queue_size

    @property
    def bot_name(self) -> str:
        return self.__bot_name

    @property
    def bot_type(self):
        return self.__bot_type

    @property
    def planner(self) -> Planner:
        return self.__planner

    @waypoint_topic.setter
    def waypoint_topic(self, waypoint_topic: str) -> None:
        self.__waypoint_topic = waypoint_topic

    @reached_topic.setter
    def reached_topic(self, reached_topic: str) -> None:
        self.__reached_topic = reached_topic

    @pos_node.setter
    def pos_node(self, pos_node: str) -> None:
        self.__pos_node = pos_node

    @pos_msg_type.setter
    def pos_msg_type(self, pos_msg_type) -> None:
        self.__pos_msg_type = pos_msg_type

    @rchd_msg_type.setter
    def rchd_msg_type(self, rchd_msg_type) -> None:
        self.__rchd_msg_type = rchd_msg_type

    @rospy_node.setter
    def rospy_node(self, rospy_node: str) -> None:
        self.__rospy_node = rospy_node

    @queue_size.setter
    def queue_size(self, queue_size: int) -> None:
        self.__queue_size = queue_size

    @bot_name.setter
    def bot_name(self, bot_name: str) -> None:
        self.__bot_name = bot_name

    @bot_type.setter
    def bot_type(self, bot_type: BotType) -> None:
        self.__bot_type = bot_type

    @planner.setter
    def planner(self, planner: Planner) -> None:
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


# ------------ PARAMETER GENERATION METHODS --------------


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
