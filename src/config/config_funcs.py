#  Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

from typing import Tuple, Optional

import yaml

from src.config.config_dicts import *
from src.config.configs import AgentConfig, MoatConfig


def __validate(cfg) -> bool:
    # TODO validate yaml file according to a schema
    return True


def get_configs(config_filename: str) \
        -> Tuple[AgentConfig, Optional[MoatConfig]]:
    with open(config_filename) as f:
        cfg = yaml.load(f)
    if not __validate(cfg):
        raise ValueError("Invalid YAML file" + config_filename)

    agent_dict = cfg['agent']
    device_dict = cfg['device']
    is_using_moat = (agent_dict.get('motion_automaton', None) is not None)

    # Make AgentConfig
    pid = agent_dict['pid']
    is_leader = (cfg['leader_pid'] == agent_dict['pid'])
    if not is_using_moat:
        moat_class = None
    else:
        # from .config_dicts import moat_class_dict
        moat_class = moat_class_dict[agent_dict['motion_automaton']]

    # from .config_dicts import mutex_handler_dict
    mh = mutex_handler_dict[cfg['mutex_handler']]

    agent_conf = AgentConfig(
        # Shared configs
        bots=cfg['num_agents'],
        # Device specific configs
        moat_class=moat_class,
        rip=device_dict['ip'],
        r_port=device_dict['rport'],
        # Agent specific configs
        pid=pid,
        is_leader=is_leader,
        mh=mh,
        plist=agent_dict['plist'],
        mh_args=[is_leader, pid]
    )

    if not is_using_moat:
        return agent_conf, None
    # else:
    # from .config_dicts import planner_dict, bot_type_dict
    moat_conf = MoatConfig(
        bot_name=device_dict['bot_name'],
        bot_type=bot_type_dict[device_dict['bot_type']],
        way_point_topic=device_dict['waypoint_topic']['topic'],
        reached_topic=device_dict['reached_topic']['topic'],
        reached_msg_type=msg_type_dict[device_dict['reached_topic']['type']],
        pos_node=device_dict['positioning_topic']['topic'],
        pos_msg_type=msg_type_dict[device_dict['positioning_topic']['type']],
        planner=planner_dict[device_dict['planner']](),
        queue_size=device_dict['queue_size'],
        ros_py_node=device_dict['ros_node_prefix']
    )
    return agent_conf, moat_conf


if __name__ == "__main__":
    import sys

    new_conf = get_configs(sys.argv[1])
    print(new_conf)
