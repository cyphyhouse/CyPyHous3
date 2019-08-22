from src.config.configs import AgentConfig, MoatConfig
from src.harness.agentThread import AgentThread
from src.motion.pos_types import pos3d, mid_pt


class LineForm(AgentThread):

    def __init__(self, agent_config: AgentConfig, moat_config: MoatConfig):
        super(LineForm, self).__init__(agent_config, moat_config)
        self.start()

    def initialize_vars(self):
        self.agent_gvh.create_ar_var('mypos', type(pos3d), self.agent_gvh.moat.position)
        self.locals['tries'] = 1

    def loop_body(self):
        while (self.locals['i'] < 10):
            self.write_to_shared('mypos', self.pid(), self.agent_gvh.moat.position)
            if not (self.pid() == 0 or self.pid() == self.num_agents() - 1):
                self.agent_gvh.moat.goTo(mid_pt(self.read_from_shared('mypos', self.pid() - 1),
                                                self.read_from_shared('mypos', self.pid() + 1)))

