import unittest

from src.config.configs import BotType
from src.config.configs import MoatConfig


class MoatConfigTest(unittest.TestCase):
    def setUp(self):
        self.config = MoatConfig('test_wp_topic', 'test_reached_topic', 'test_wp_node', 'drone1', 1, BotType.CAR,
                                 'test_pos_node')
        pass

    def test_setup(self):
        self.assertEqual(self.config.waypoint_topic, 'test_wp_topic')
        self.assertEqual(self.config.reached_topic, 'test_reached_topic')
        self.assertEqual(self.config.rospy_node, 'test_wp_node')
        self.assertEqual(self.config.bot_type, BotType.CAR)


if __name__ == '__main__':
    unittest.main()
