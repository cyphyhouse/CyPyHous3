import math

from src.config.configs import AgentConfig, MoatConfig
from src.harness.agentThread import AgentThread
from src.motion.pos_types import pos3d


class ShapeForm(AgentThread):

    def __init__(self, agent_config: AgentConfig, moat_config: MoatConfig):
        super(ShapeForm, self).__init__(agent_config, moat_config)

    def initialize_vars(self):
        self.agent_gvh.create_ar_var('mypos', type(pos3d), self.agent_gvh.moat.position)

    def loop_body(self):
        if self.num_agents() not in [x**2 for x in range(1, 6)]:
            self.trystop()  # Avoid non square number of agents
            return
        self.write_to_shared('mypos', self.pid(), self.agent_gvh.moat.position)
        if self.pid() in get_corners(self.num_agents()):
            self.agent_gvh.moat.goTo(get_corner_pos(self.pid(), self.num_agents()))
        else:
            self.agent_gvh.moat.goTo(
                mid_pt_of_list(
                    [self.read_from_shared('mypos', i) for i in get_nbrs(self.pid(), self.num_agents())]
                ))


def mid_pt_of_list(poslist):
    return pos3d(sum([p.x for p in poslist]) / len(poslist), sum([p.y for p in poslist]) / len(poslist),
                 sum([p.z for p in poslist]) / len(poslist))


def get_corners(n):
    assert math.sqrt(n).is_integer()

    w = int(math.sqrt(n))
    corner_coord = [(0, 0), (0, w-1), (w-1, 0), (w-1, w-1)]
    corners = set([x*w + y for x, y in corner_coord])
    return corners


def get_corner_pos(pid, num_agents):
    assert math.sqrt(num_agents).is_integer()
    w = int(math.sqrt(num_agents))
    x, y = (pid // w, pid % w)

    space = 2.5
    return pos3d((x - (w-1) / 2)*space,
                 (y - (w-1) / 2)*space,
                 1 + 2 * space if x==0 and y==0 else (1 + space if x != y else 1 ))


def get_nbrs(pid, num_agents):
    assert math.sqrt(num_agents).is_integer()

    w = int(math.sqrt(num_agents))
    x, y = (pid // w, pid % w)

    if x == 0 or x == w-1:
        all_nbrs = [(x, y-1), (x, y+1)]
    elif y == 0 or y == w-1:
        all_nbrs = [(x-1, y), (x+1, y)]
    else:
        all_nbrs = [(x-1, y), (x+1, y), (x, y-1), (x, y+1)]

    prop_nbrs = [(nx, ny) for nx, ny in all_nbrs if 0 <= nx < w and 0 <= ny < w]
    return set([nx*w + ny for nx, ny in prop_nbrs])
