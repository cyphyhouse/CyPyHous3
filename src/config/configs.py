# Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

import typing as tp

import src.datatypes.robot as robot
import src.motion.abstract.planner as pl
import src.motion.simplePlanner as sp


class AgentConfig(object):
    """
    agent configuration object

    __pid : unique integer identifier of the agent
    __rip : receiver ip
    __r_port : receiver (listener) port
    __plist : list of ports (used when multiple instances of Koord program running on same robot)
    __mutex_handler : mutual exclusion handler object
    __is_leader : property determining whether the agent is leader (used in conflict resolution in various cases
    __moat_class : motion automaton specification
    __send_ips: IPs with corresponding ports of other participating systems (default is broadcast)
    """

    def __init__(self, pid: int, bots: int, rip: str, r_port: int, plist: tp.Union[tp.List[int], None] = None,
                 mh: tp.Union[None, classmethod] = None, is_leader=False,
                 moat_class: tp.Union[None, classmethod] = None,
                 mh_args: tp.Union[tp.List, None] = None, send_ips: tp.List[tp.Tuple[str, int]] = None):
        """

        :param pid : unique integer identifier
        :type pid : int

        :param bots : number of robots
        :type bots : int

        :param rip : receiver ip
        :type rip : str

        :param r_port : listener port
        :type r_port : int

        :param plist : list of listener ports
        :type plist : list(int)

        :param mh : mutual handler
        :type mh : class method, None

        :param is_leader : boolean indicating whether robot is leader
        :type is_leader : bool

        :param moat_class : motion automaton class
        :type moat_class : class method, None

        :param mh_args : mutual handler arguments
        :type mh_args : list

        :param send_ips : list of ips participating in the application, with corresponding ports
        :type send_ips : List[Tuple[str,int]]
        """
        if plist is None:
            plist = []
        if mh_args is None:
            mh_args = []
        if mh is not None:
            self.__mutex_handler = mh(*mh_args)
        else:
            self.__mutex_handler = None
        if send_ips is None:
            send_ips = []
        self.__pid = pid
        self.__bots = bots
        self.__rip = rip
        self.__r_port = r_port
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
    def r_port(self) -> int:
        return self.__r_port

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

    @property
    def send_ips(self) -> tp.List[tp.Tuple[str, int]]:
        return self.__send_ips

    @pid.setter
    def pid(self, pid: int) -> None:
        self.__pid = pid

    @bots.setter
    def bots(self, bots: int) -> None:
        self.__bots = bots

    @rip.setter
    def rip(self, rip: str) -> None:
        self.__rip = rip

    @r_port.setter
    def r_port(self, r_port: int) -> None:
        self.__r_port = r_port

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

    @send_ips.setter
    def send_ips(self, send_ips: tp.List[tp.Tuple[str, int]]):
        self.__send_ips = send_ips

    def __repr__(self):
        """
        string representation
        """
        s = ""
        s += "pid" + ":" + str(self.__pid) + "\n"
        s += "bots" + ":" + str(self.__bots) + "\n"
        s += "rip" + ":" + str(self.__rip) + "\n"
        s += "r_port" + ":" + str(self.__r_port) + "\n"
        s += "plist" + ":" + str(self.__plist) + "\n"
        s += "mutex_handler" + ":" + str(self.__mutex_handler) + "\n"
        s += "is_leader" + ":" + str(self.__is_leader) + "\n"
        s += "moat_class" + ":" + str(self.__moat_class) + "\n"
        s += "ip_port_list" + ":" + str(self.__send_ips)
        return s

    # ----------------- OPERATIONS --------------------

    def __eq__(self, other):
        """
        checking equality
        """
        return self.pid == other.pid \
               and self.bots == other.bots \
               and self.rip == other.rip \
               and self.r_port == other.r_port \
               and self.plist == other.plist \
               and self.mutex_handler == other.mutex_handler \
               and self.is_leader == other.is_leader \
               and self.moat_class == other.moat_class


class MoatConfig(object):
    """
    motion configuration object
    __way_point_topic : msg type to publish to set way points for controller
    __reached_topic : reached or not
    __ros_py_node : RosPy node corresponding to motion automaton
    __bot_name : unique string name for robot
    __queue_size : number of ros messages stored
    __bot_type : bot type (to determine planner , etc)
    __pos_node : position node (ros)
    __pos_msg_type : position message type (ros)
    __reached_msg_type : reached message type (ros)
    __planner : path planner
    """

    def __init__(self, way_point_topic: str, reached_topic: str, ros_py_node: str, bot_name: str, queue_size: int,
                 bot_type: robot.BotType, pos_node: str, pos_msg_type=None, reached_msg_type=None,
                 planner: pl.Planner = sp.SimplePlanner()):
        """
        :param way_point_topic: way point message ros topic
        :type way_point_topic: str

        :param reached_topic: reached message ros topic
        :type reached_topic: str

        :param ros_py_node: RosPy node name
        :type ros_py_node: str

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

        :param reached_msg_type: reached message type
        :type reached_msg_type: (determined by ros message)

        :param planner: path planner
        :type planner: Planner
        """
        self.__way_point_topic = way_point_topic
        self.__reached_topic = reached_topic
        self.__pos_node = pos_node
        self.__pos_msg_type = pos_msg_type
        self.__reached_msg_type = reached_msg_type
        self.__ros_py_node = ros_py_node
        self.__bot_name = bot_name
        self.__queue_size = queue_size
        self.__bot_type = bot_type
        self.__planner = planner

    # ------------ MEMBER ACCESS METHODS --------------

    @property
    def way_point_topic(self) -> str:
        return self.__way_point_topic

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
    def reached_msg_type(self):
        return self.__reached_msg_type

    @property
    def ros_py_node(self) -> str:
        return self.__ros_py_node

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
    def planner(self) -> pl.Planner:
        return self.__planner

    @way_point_topic.setter
    def way_point_topic(self, way_point_topic: str) -> None:
        self.__way_point_topic = way_point_topic

    @reached_topic.setter
    def reached_topic(self, reached_topic: str) -> None:
        self.__reached_topic = reached_topic

    @pos_node.setter
    def pos_node(self, pos_node: str) -> None:
        self.__pos_node = pos_node

    @pos_msg_type.setter
    def pos_msg_type(self, pos_msg_type) -> None:
        self.__pos_msg_type = pos_msg_type

    @reached_msg_type.setter
    def reached_msg_type(self, reached_msg_type) -> None:
        self.__reached_msg_type = reached_msg_type

    @ros_py_node.setter
    def ros_py_node(self, ros_py_node: str) -> None:
        self.__ros_py_node = ros_py_node

    @queue_size.setter
    def queue_size(self, queue_size: int) -> None:
        self.__queue_size = queue_size

    @bot_name.setter
    def bot_name(self, bot_name: str) -> None:
        self.__bot_name = bot_name

    @bot_type.setter
    def bot_type(self, bot_type: robot.BotType) -> None:
        self.__bot_type = bot_type

    @planner.setter
    def planner(self, planner: pl.Planner) -> None:
        self.__planner = planner

    def __repr__(self):
        s = ""
        s += "way_point_topic" + ":" + str(self.way_point_topic) + "\n"
        s += "reached_topic" + ":" + str(self.reached_topic) + "\n"
        s += "pos_msg_type" + ":" + str(self.pos_msg_type) + "\n"
        s += "reached_msg_type" + ":" + str(self.reached_msg_type) + "\n"
        s += "ros_py_node" + ":" + str(self.ros_py_node) + "\n"
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
    return config.ros_py_node + '/' + config.reached_topic, config.reached_msg_type


def gen_way_point_params(config: MoatConfig):
    """
    TODO: write unit test
    generate way point message parameters from motion configuration

    :param config: Motion automaton Config
    :type config: MoatConfig

    :return: way point parameters
    :rtype: Tuple
    """
    return config.ros_py_node + '/' + config.way_point_topic, config.pos_msg_type
