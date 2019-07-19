import unittest

from src.harness.configs import BotType
from src.harness.configs import MoatConfig, gen_reached_params, gen_positioning_params


class MoatConfigTest(unittest.TestCase):
    def setUp(self):
        self.config = MoatConfig('test_wp_topic', 'test_reached_topic', 'test_wp_node',
                                 ('test_node_name/test_vehicle/pose', 'test_msg_type'),
                                 ('test_reached_topic', 'test_msg_type'), 'test_vehicle', 1, BotType.CAR)
        pass

    def test_setup(self):
        self.assertEqual(self.config.waypoint_topic, 'test_wp_topic')
        self.assertEqual(self.config.reached_topic, 'test_reached_topic')
        self.assertEqual(self.config.rospy_node, 'test_wp_node')
        self.assertEqual(self.config.bot_type, BotType.CAR)
        self.assertEqual(self.config.reached_params, gen_reached_params(self.config.reached_topic, 'test_msg_type'))
        self.assertEqual(self.config.positioning_params,
                         gen_positioning_params('test_node_name/', self.config.bot_name, 'test_msg_type'))


if __name__ == '__main__':
    unittest.main()
