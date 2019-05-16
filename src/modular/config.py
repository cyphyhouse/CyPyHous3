import point

def get_data(pid,configfile):
    f = open(configfile,'r').readlines()
    fields = ['pid', 'initpos', 'xvel', 'yvel', 'xacc', 'yacc', 'timetic', 'speed']
    dict1 = dict(zip(fields,f[pid].strip().split(';')))
    return dict1

def pos(pid,configfile):
    p = get_data(pid,configfile)['initpos'].replace(")","").replace("(","").split(',')
    return point.point(float(p[0]),float(p[1]),float(p[2]))

def xvel(pid, configfile):
    return float(get_data(pid, configfile)['xvel'])


def yvel(pid, configfile):
    return float(get_data(pid, configfile)['yvel'])


def xacc(pid, configfile):
    return float(get_data(pid, configfile)['xacc'])


def yacc(pid, configfile):
    return float(get_data(pid, configfile)['yacc'])


def timetic(pid, configfile):
    return float(get_data(pid, configfile)['timetic'])


def speed(pid, configfile):
    return float(get_data(pid, configfile)['speed'])

