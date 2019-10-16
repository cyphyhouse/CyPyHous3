from src.config.configs import AgentConfig, MoatConfig
from src.harness.agentThread import AgentThread
from src.motion.pos_types import pos3d
import time


class MapApp(AgentThread):

    def __init__(self, agent_config: AgentConfig, moat_config: MoatConfig):
        super(MapApp, self).__init__(agent_config, moat_config)

    def initialize_vars(self):
        self.locals['new_point'] = True
        pass

    def loop_body(self):
        while not self.stopped():
            time.sleep(0.1)
            if self.locals['new_point'] :
                self.locals['new_point'] = False
                return

            if not self.locals['new_point']:
                self.stop()
                return

