# configuration objects
from enum import Enum, unique
from typing import Union

from src.functionality.mutex_handler import BaseMutexHandler


class AgentConfig(object):
    def __init__(self, pid: int, bots: int, rip: str, rport: int, plist: list = [],
                 mh: Union[BaseMutexHandler, None] = None, is_leader=False, moat_class=None):
        self.pid = pid
        self.bots = bots
        self.rip = rip
        self.rport = rport
        self.plist = plist
        self.mutex_handler = mh
        self.is_leader = is_leader
        self.moat_class = moat_class

    def __repr__(self):
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


@unique
class BotType(Enum):
    """
    add more bot types here
    """
    QUAD = 1
    CAR = 2


class Topic(object):

    def __init__(self, name, ttype):
        self.name = name
        self.type = ttype


class MoatConfig(object):
    from src.motion.planner import Planner
    from src.motion.simpleplanner import SimplePlanner

    def __init__(self, waypoint_topic: str, reached_topic: str, rospy_node: str, bot_name: str, queue_size: int,
                 bot_type: BotType, pos_node: str, pos_msg_type=None, rchd_msg_type=None,
                 planner: Planner = SimplePlanner()):
        """

        :param waypoint_topic:
        :param reached_topic:
        :param rospy_node:
        :param bot_name:
        :param queue_size:
        :param bot_type:
        :param pos_node:
        :param pos_msg_type:
        :param rchd_msg_type:
        :param planner:
        """
        self.waypoint_topic = waypoint_topic
        self.reached_topic = reached_topic
        self.pos_node = pos_node
        self.pos_msg_type = pos_msg_type
        self.rchd_msg_type = rchd_msg_type
        self.rospy_node = rospy_node
        self.bot_name = bot_name
        self.queue_size = queue_size
        self.bot_type = bot_type
        self.planner = planner

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


def gen_positioning_params(config: MoatConfig):
    return config.pos_node + config.bot_name + '/pose', config.pos_msg_type


def gen_reached_params(config: MoatConfig):
    return config.reached_topic, config.rchd_msg_type


# TODO generate default simulation moat car config
# TODO generate detault simulation moat drone config

def default_car_moat_config(bot_name) -> MoatConfig:
    rospy_node = 'quad_wp_node'
    bot_type = BotType.CAR
    waypoint_topic = 'Waypoint'
    reached_topic = '/Reached'

    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String

    queue_size = 1
    bot_name = bot_name
    pos_node = '/vrpn_client_node/'
    pos_msg_type = PoseStamped
    rchd_msg_type = String
    from src.motion.simpleplanner import SimplePlanner

    return MoatConfig(waypoint_topic, reached_topic, rospy_node, bot_name, queue_size, bot_type, pos_node, pos_msg_type,
                      rchd_msg_type, SimplePlanner())


def default_qc_moat_config(bot_name) -> MoatConfig:
    rospy_node = 'quad_wp_node'
    bot_type = BotType.QUAD
    waypoint_topic = 'Waypoint'
    reached_topic = '/Reached'

    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String

    queue_size = 1
    bot_name = bot_name
    pos_node = '/vrpn_client_node/'
    pos_msg_type = PoseStamped
    rchd_msg_type = String
    from src.motion.simpleplanner import SimplePlanner

    return MoatConfig(waypoint_topic, reached_topic, rospy_node, bot_name, queue_size, bot_type, pos_node, pos_msg_type,
                      rchd_msg_type, SimplePlanner())