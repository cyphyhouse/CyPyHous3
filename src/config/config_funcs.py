from typing import Tuple, Optional
import yaml

from src.config.configs import AgentConfig, MoatConfig


def __validate(cfg) -> bool:
    # TODO validate yaml file according to a schema
    return True


def get_configs(config_filename: str) -> Tuple[AgentConfig, MoatConfig]:
    with open(config_filename) as f:
        cfg = yaml.load(f)
    if not __validate(cfg):
        raise ValueError("Invalid YAML file" + config_filename)

    # Make AgentConfig
    from .config_dicts import mutex_handler_dict, moat_class_dict
    agent_dict = cfg['agent']
    device_dict = cfg['device']

    pid = agent_dict['pid']
    is_leader=(cfg['leader_pid'] == agent_dict['pid'])

    agent_conf = AgentConfig(
        # Shared configs
        bots=cfg['num_agents'],
        # Device specific configs
        moat_class=moat_class_dict[agent_dict['motion_automaton']],  # TODO choose Motion Automaton only when necessary
        rip=device_dict['ip'],
        rport=device_dict['port'],
        # Agent specific configs
        pid=pid,
        is_leader=is_leader,
        mh=mutex_handler_dict[cfg['mutex_handler']](pid, is_leader),
        plist=agent_dict['plist'],
    )

    from .config_dicts import planner_dict, bot_type_dict, msg_type_dict
    moat_conf = MoatConfig(
        bot_name=device_dict['bot_name'],
        bot_type=bot_type_dict[device_dict['bot_type']],
        waypoint_topic=device_dict['waypoint_topic']['topic'],
        reached_topic=device_dict['reached_topic']['topic'],
        rchd_msg_type=device_dict['reached_topic']['type'],
        pos_node=device_dict['positioning_topic']['topic'],
        pos_msg_type=device_dict['positioning_topic']['type'],
        planner=planner_dict[device_dict['planner']],
        queue_size=device_dict['queue_size'],
        rospy_node=device_dict['ros_node_prefix']
    )

    return agent_conf, moat_conf


def get_configs_old(configfilename, agentnum, moatnum):
    f = open(configfilename)
    cfg = yaml.load(f)
    agent_config = None
    moat_config = None
    if agentnum >= 0:
        agent_config = mk_agent_config(cfg['agents'][agentnum])
    if moatnum >=0:
        moat_config = mk_moat_config(cfg['motion_automaton'][moatnum])
    return [agent_config, moat_config]


def mk_agent_config(agent_dict):
    from src.config.config_dicts import mutex_handler_dict, moat_class_dict
    agent_dict['mutex_handler'] = mutex_handler_dict[agent_dict['mutex_handler']]
    agent_dict['moat_class'] = moat_class_dict[agent_dict['moat_class']]
    a = AgentConfig(agent_dict['pid'], agent_dict['bots'], agent_dict['rip'], agent_dict['rport'], agent_dict['plist'],
                    agent_dict['mutex_handler'](agent_dict['pid'], agent_dict['is_leader']), agent_dict['is_leader'],
                    agent_dict['moat_class'])
    return a


def mk_moat_config(moat_dict):
    from src.config.config_dicts import planner_dict, bot_type_dict, msg_type_dict
    moat_dict['planner'] = planner_dict[moat_dict['planner']]
    moat_dict['pos_msg_type'] = msg_type_dict[moat_dict['pos_msg_type']]
    moat_dict['rchd_msg_type'] = msg_type_dict[moat_dict['rchd_msg_type']]
    moat_dict['bot_type'] = bot_type_dict[moat_dict['bot_type']]
    return MoatConfig(**moat_dict)


if __name__ == "__main__":
    import sys

    new_conf = get_configs(sys.argv[1])
    old_conf = get_configs_old(sys.argv[2], 1, 1)

    assert new_conf[0].__dict__.keys() == old_conf[0].__dict__.keys()
    assert all(new_conf[0].__dict__[k] == old_conf[0].__dict__[k] \
            for k in new_conf[0].__dict__.keys() if k != 'mutex_handler')

    assert new_conf[1].__dict__ == old_conf[1].__dict__, \
        "New: " + str(new_conf[1].__dict__) + "\n" + \
        "Old: " + str(old_conf[1].__dict__) + "\n"
