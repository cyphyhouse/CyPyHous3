# Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

import unittest

from src.config.configs import MoatConfig
from src.datatypes.robot import BotType


class TestMoatConfig(unittest.TestCase):
    """
    testing the agent config member access methods
    __moat_class : motion automaton specification
    """

    def setUp(self):
        self.mc = MoatConfig("wpt", "rct", "rpn", "bn", 2, BotType.CAR, "pn", 5, 6)

    def test_waypoint_topic(self):
        self.assertEqual(self.mc.waypoint_topic, "wpt", "test waypoint topic")

    def test_reached_topic(self):
        self.assertEqual(self.mc.reached_topic, "rct", "test reached topic")

    def test_rospy_node(self):
        self.assertEqual(self.mc.rospy_node, "rpn", "test rospy node")

    def test_bot_name(self):
        self.assertEqual(self.mc.bot_name, "bn", "test bot name")

    def test_queue_size(self):
        self.assertEqual(self.mc.queue_size, 2, "test queue size")

    def test_bot_type(self):
        self.assertEqual(self.mc.bot_type, BotType.CAR, "test bot type")

    def test_pos_node(self):
        self.assertEqual(self.mc.pos_node, "pn", "test pos node")

    def test_pos_msg_type(self):
        self.assertEqual(self.mc.pos_msg_type, 5, "test pos message type")

    def test_rchd_msg_type(self):
        self.assertEqual(self.mc.rchd_msg_type, 6, "test reached message type")

    def test_planner(self):
        self.assertEqual(self.mc.planner, None, "test planner")


if __name__ == '__main__':
    unittest.main()
