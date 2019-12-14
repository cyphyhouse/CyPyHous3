#  Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

from typing import Tuple, Optional

import yamale
import yaml

import src.config.config_dicts as config_dicts
import src.config.configs as configs


def __validate(yaml_file: str, schema_file: str) -> bool:
    """
    validation function
    :param yaml_file: config filename
    :type yaml_file: str

    :param schema_file: reference valid schema
    :type schema_file: str

    :return: whether or not config is valid
    :rtype: bool
    """
    schema = yamale.make_schema(schema_file)
    data = yamale.make_data(yaml_file)
    yamale.validate(schema, data)
    return True


def get_configs(config_filename: str, schema_filename: str) \
        -> Tuple[configs.AgentConfig, Optional[configs.MoatConfig]]:
    """

    :param config_filename: config filename
    :type config_filename: str

    :param schema_filename: reference schema filename
    :type schema_filename: str

    :return: configuration objects
    :rtype: Tuple[AgentConfig, MoatConfig)
    """
    with open(config_filename) as f:
        cfg = yaml.safe_load(f)
    if not __validate(config_filename, schema_filename):
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
        moat_class = config_dicts.moat_class_dict[agent_dict['motion_automaton']]

    mh = config_dicts.mutex_handler_dict[cfg['mutex_handler']]

    agent_conf = configs.AgentConfig(
        # Shared configs
        bots=cfg['num_agents'],

        # Device specific configs
        moat_class=moat_class,
        rip=device_dict['ip'],
        r_port=device_dict['r_port'],

        # Agent specific configs
        pid=pid,
        is_leader=is_leader,
        mh=mh,
        plist=agent_dict['plist'],
        mh_args=[is_leader, pid]
    )

    if not is_using_moat:
        return agent_conf, None

    moat_conf = configs.MoatConfig(
        bot_name=device_dict['bot_name'],
        bot_type=config_dicts.bot_type_dict[device_dict['bot_type']],
        way_point_topic=device_dict['waypoint_topic']['topic'],
        reached_topic=device_dict['reached_topic']['topic'],
        reached_msg_type=config_dicts.msg_type_dict[device_dict['reached_topic']['type']],
        pos_node=device_dict['positioning_topic']['topic'],
        pos_msg_type=config_dicts.msg_type_dict[device_dict['positioning_topic']['type']],
        planner=config_dicts.planner_dict[device_dict['planner']](),
        queue_size=device_dict['queue_size'],
        ros_py_node=device_dict['ros_node_prefix']
    )
    return agent_conf, moat_conf


if __name__ == "__main__":
    import sys

    new_conf = get_configs(sys.argv[1], sys.argv[2])
    print(new_conf)
