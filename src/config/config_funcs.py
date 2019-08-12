from typing import Dict, Tuple, Optional

from src.config.configs import AgentConfig, MoatConfig
from src.config.config_dicts import *


def __validate(cfg) -> bool:
    # TODO validate yaml file according to a schema
    return True


def get_configs(cfg: Dict) \
        -> Tuple[AgentConfig, Optional[MoatConfig]]:
    if not __validate(cfg):
        raise ValueError("Invalid local config value")

    agent_dict = cfg['agent']
    device_dict = cfg['device']
    is_using_moat = (agent_dict.get('motion_automaton', None) is not None)

    # Make AgentConfig
    pid = agent_dict['pid']
    is_leader = (cfg['leader_pid'] == agent_dict['pid'])
    if not is_using_moat:
        moat_class = None
    else:
        moat_class = moat_class_dict[agent_dict['motion_automaton']]

    mh = mutex_handler_dict[cfg['mutex_handler']]

    agent_conf = AgentConfig(
        # Shared configs
        bots=cfg['num_agents'],
        # Device specific configs
        moat_class=moat_class,
        rip=device_dict['ip'],
        rport=device_dict['port'],
        # Agent specific configs
        pid=pid,
        is_leader=is_leader,
        mh=mh,
        plist=agent_dict['plist'],
        mhargs=[is_leader,pid]
    )

    if not is_using_moat:
        return agent_conf, None
    # else:
    moat_conf = MoatConfig(
        bot_name=device_dict['bot_name'],
        bot_type=bot_type_dict[device_dict['bot_type']],
        waypoint_topic=device_dict['waypoint_topic']['topic'],
        reached_topic=device_dict['reached_topic']['topic'],
        rchd_msg_type=msg_type_dict[device_dict['reached_topic']['type']],
        pos_node=device_dict['positioning_topic']['topic'],
        pos_msg_type=msg_type_dict[device_dict['positioning_topic']['type']],
        planner=planner_dict[device_dict['planner']](),
        queue_size=device_dict['queue_size'],
        rospy_node=device_dict['ros_node_prefix']
    )
    return agent_conf, moat_conf


if __name__ == "__main__":
    import sys
    import yaml

    with open(sys.argv[1], 'r') as f:
        cfg = yaml.safe_load(f)
        new_conf = get_configs(cfg)
    print(new_conf)
