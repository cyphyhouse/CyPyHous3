from unittest import TestCase

from src.agent_thread import AgentThread
from src.config.configs import AgentConfig


class DummyAgentThread(AgentThread):
    def _initialize_vars(self) -> None:
        pass

    def _loop_body(self) -> None:
        pass


class AgentThreadTest(TestCase):
    def setUp(self) -> None:
        try:
            import rospy
            # Avoid spawning a ROS node and use `time` module and system time
            rospy.rostime.switch_to_wallclock()
            rospy.rostime.set_rostime_initialized(True)
        except ImportError:
            pass

    def test_stop(self):
        agent_config = AgentConfig(0, 1, "127.1.1.1", 2000, [2000], None, True, None, None)
        agent_th = DummyAgentThread(agent_config)
        agent_th.start()

        agent_th.stop()
        n = 3
        agent_th.join(timeout=n*agent_th.delta)
        self.assertFalse(agent_th.is_alive(),
                         "Agent thread is still alive after %d delta cycles" % n)
        agent_th.join()
