# configuration objects
from enum import Enum
from typing import Union

from src.functionality.mutex_handler import BaseMutexHandler
from src.motion.planner import Planner



class AgentConfig(object):
    def __init__(self, pid: int, bots: int, rip: str, rport: int, plist: list = [],
                 mh: Union[BaseMutexHandler, None] = None, is_leader = False, moat_class = None):
        self.pid = pid
        self.bots = bots
        self.rip = rip
        self.rport = rport
        self.plist = plist
        self.mutex_handler = mh
        self.is_leader = is_leader
        self.moat_class = moat_class


class BotType(Enum):
    """
    add more bot types here
    """
    QUAD = 1
    CAR = 2


class MoatConfig(object):
    from src.motion.simpleplanner import SimplePlanner

    def __init__(self, waypoint_topic: str, reached_topic: str, rospy_node: str, positioning_params: tuple,
                 reached_params: tuple, bot_name: str, queue_size: int, bot_type: BotType,
                 pub_msg_type=None, planner: Planner = SimplePlanner()):
        """

        :param waypoint_topic:
        :param reached_topic:
        :param rospy_node:
        :param positioning_params:
        :param bot_name:
        :param queue_size:
        :param bot_type:
        :param reached_params:
        :param msg_type:
        :param planner:
        """
        self.waypoint_topic = waypoint_topic
        self.reached_topic = reached_topic
        self.reached_params = reached_params
        self.positioning_params = positioning_params
        self.pub_msg_type = pub_msg_type
        self.rospy_node = rospy_node
        self.bot_name = bot_name
        self.queue_size = queue_size
        self.bot_type = bot_type
        self.planner = planner
        self.moat_class = moat_class


def gen_positioning_params(node_name, bot_name, msg_type):
    return node_name + bot_name + '/pose', msg_type


def gen_reached_params(reached_topic, msg_type):
    return reached_topic, msg_type


# TODO generate default simulation moat car config
# TODO generate detault simulation moat drone config

def default_car_moat_config(bot_name) -> MoatConfig:
    rospy_node = 'quad_wp_node'
    bot_type = BotType.CAR
    waypoint_topic = 'Waypoint'
    reached_topic = '/Reached'

    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String

    pub_msg_type = PoseStamped
    queue_size = 1
    bot_name = bot_name
    positioning_params = gen_positioning_params('/vrpn_client_node/', bot_name, PoseStamped)
    reached_params = gen_reached_params(reached_topic, String)
    return MoatConfig(waypoint_topic, reached_topic, rospy_node, positioning_params, reached_params, bot_name,
                      queue_size, bot_type, pub_msg_type, SimplePlanner())


def default_qc_moat_config(bot_name) -> MoatConfig:
    rospy_node = 'quad_wp_node'
    bot_type = BotType.CAR
    waypoint_topic = 'Waypoint'
    reached_topic = '/Reached'

    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String

    pub_msg_type = PoseStamped
    queue_size = 1
    bot_name = bot_name
    positioning_params = gen_positioning_params('/vrpn_client_node/', bot_name, PoseStamped)
    reached_params = gen_reached_params(reached_topic, String)
    return MoatConfig(waypoint_topic, reached_topic, rospy_node, positioning_params, reached_params, bot_name,
                      queue_size, bot_type, pub_msg_type, SimplePlanner())
