import math

from src.config.configs import AgentConfig, MoatConfig
from src.harness.agentThread import AgentThread
from src.motion.pos_types import pos3d


class Shapeform(AgentThread):

    def __init__(self, agent_config: AgentConfig, moat_config: MoatConfig):
        super(Shapeform, self).__init__(agent_config, moat_config)
        self.start()

    def initialize_vars(self):
        self.agent_gvh.create_ar_var('mypos', type(pos3d), self.agent_gvh.moat.position)
        self.locals['tries'] = 1
        self.locals['gridmap'] = get_gridmap(self.num_agents())
        self.locals['corners'] = get_corners(self.num_agents())
        self.locals['nbrs'] = get_nbrs(self.pid(),self.locals['gridmap'])

    def loop_body(self):
        self.write_to_shared('mypos', self.pid(), self.agent_gvh.moat.position)
        if not self.pid() in self.locals['corners']:
            self.agent_gvh.moat.goTo(mid_pt_of_list([self.read_from_shared('mypos', i) for i in self.locals['nbrs']]))
            self.locals['tries'] += 1

        if self.locals['tries'] == 100:
            self.trystop()


def mid_pt_of_list(poslist):
    return pos3d(sum([p.x for p in poslist]) / len(poslist), sum([p.y for p in poslist]) / len(poslist),
                 sum([p.z for p in poslist]) / len(poslist))


def get_corners(n):
    corners = []
    if math.sqrt(n).is_integer():
        max = int(math.sqrt(n)) - 1
        min = 0
        corners = [(min,min), (min,max), (max,min), (max,max)
    return corners

def get_gridmap(n):
    i = 0
    gridmap = []
    if math.sqrt(n).is_integer():
        max = int(math.sqrt(n))
        min = 0
        for j in range(min, max ):
            for k in range(min,max):
                gridmap.append((i, (j, k)))
                i = i + 1
        return gridmap

    return None


def pid2d(pid, gridmap):
    for i in gridmap:
        if pid == i[0]:
            return i[1]
    return None


def prop_nbrs(p):
    x1 = p[0] + 1
    x2 = p[0] - 1
    y1 = p[1] + 1
    y2 = p[1] - 1
    return [(p[0], y1), (p[0], y2), (x1, p[1]), (x2, p[1])]


def get_nbrs(pid, gridmap):
    p = pid2d(pid, gridmap)
    n = prop_nbrs(p)
    gridlist = [p[1] for p in gridmap]
    return list(set(gridlist).intersection(n))


