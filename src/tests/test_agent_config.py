import unittest

from src.functionality.base_mutex_handler import BaseMutexHandler
from src.config.configs import AgentConfig


class AgentConfigTest(unittest.TestCase):
    def setUp(self):
        self.config = AgentConfig(0, 1, "", 2000, [2001, 2002])
        pass

    def test_setup(self):
        self.assertEqual(self.config.pid, 0)
        self.assertEqual(self.config.bots, 1)
        self.assertEqual(self.config.rip, "")
        self.assertEqual(self.config.rport, 2000)
        self.assertEqual(self.config.plist[0], 2001)

    def test_bot_change(self):
        self.config.bots = 2
        self.assertEqual(self.config.bots, 2)

    def test_base_mutex_handler(self):
        self.config.mutex_handler = BaseMutexHandler(True, 0)
        self.assertEqual(self.config.mutex_handler.pid, 0)
        self.config.mutex_handler.stop()



