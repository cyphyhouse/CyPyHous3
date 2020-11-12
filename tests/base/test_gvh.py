# Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

import unittest

from src.config.configs import AgentConfig, MoatConfig
from src.datatypes.message_types import MsgType
from src.datatypes.robot import BotType
from src.harness.gvh import gvh
from src.objects.message import Message


class TestGvh(unittest.TestCase):
    """
    testing gvh member access methods
    """

    def setUp(self):
        self.mc = MoatConfig("wpt", "rct", "rpn", "bn", 2, BotType.CAR, "pn", 5, 6)
        self.ac = AgentConfig(0, 3, "127.1.1.1", 2000, [2000], None, True, None, None)
        self.tgvh = gvh(self.ac, self.mc)

    def test_is_leader(self):
        self.assertEqual(self.tgvh.is_leader, self.ac.is_leader, "testing is leader")

    def test_is_alive(self):
        self.assertTrue(self.tgvh.is_alive, "testing is alive")
        self.tgvh.is_alive = False
        self.assertFalse(self.tgvh.is_alive, "testing is alive")

    def test_msg_list(self):
        self.assertEqual(self.tgvh.msg_list, [], "testing msg list")
        self.tgvh.msg_list.append(1)
        self.assertEqual(self.tgvh.msg_list[0], 1, "testing msg_list")

    def test_recv_msg_list(self):
        self.assertEqual(self.tgvh.recv_msg_list, [], "testing received msg list")
        self.tgvh.recv_msg_list.append(1)

    def test_multi_flag(self):
        self.assertTrue(self.tgvh.get_multi_flag(), "testing whether multiple messages need to be sent")

    def test_single_args(self):
        msg = Message(1, MsgType.STOP_CONFIRM, 1, 1.0)
        self.assertEqual(self.tgvh.mk_single_msg_args(msg), [msg, "<broadcast>", self.tgvh.r_port],
                         "testing single message args")

    def test_multi_args(self):
        msg = Message(1, MsgType.STOP_CONFIRM, 1, 1.0)
        self.assertEqual(self.tgvh.mk_multi_msg_args(msg), [[msg, "<broadcast>", 2000]], "testing multi message args")


if __name__ == '__main__':
    unittest.main()
