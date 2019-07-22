import yaml

from src.config.configs import AgentConfig, MoatConfig


def get_configs(configfilename, agentnum, moatnum):
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
    list_keys = [a for a in moat_dict.keys()]
    moat_dict[list_keys[9]] = planner_dict[moat_dict[list_keys[9]]]
    moat_dict[list_keys[7]] = msg_type_dict[moat_dict[list_keys[7]]]
    moat_dict[list_keys[8]] = msg_type_dict[moat_dict[list_keys[8]]]
    moat_dict[list_keys[5]] = bot_type_dict[moat_dict[list_keys[5]]]
    moat_config_params = [moat_dict[list_keys[i]] for i in range(len(list_keys))]
    return MoatConfig(*moat_config_params)



# print(cfg['motion_automaton'][0])
# a = mk_moat_config(cfg['motion_automaton'][0])
#a = get_configs('/Users/mim/CyPyHous3/src/config/simple.yml',-1,-1)
#print(a)
