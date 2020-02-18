# Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

import unittest

import src.datatypes.message_types as mt
import src.harness.msg_create as mc


class TestMsgCreate(unittest.TestCase):

    def setUp(self):
        pass

    def test_round_update_msg_create(self):
        self.assertEqual(mc.round_update_msg_create(1, 1, 1.0).message_type, mt.MsgType.ROUND_UPDATE,
                         "test round update message create")

    def test_round_update_msg_confirm_create(self):
        self.assertEqual(mc.round_update_msg_confirm_create(1, 1, 1, 1.0).message_type, mt.MsgType.ROUND_UPDATE_CONFIRM,
                         "test round update confirm message create")

    def test_stop_msg_create(self):
        self.assertEqual(mc.stop_msg_create(1, 1, 1.0).message_type, mt.MsgType.STOP, "test stop message create")

    def test_stop_msg_confirm_create(self):
        self.assertEqual(mc.stop_msg_confirm_create(1, 1, 1.0).message_type, mt.MsgType.STOP_CONFIRM,
                         "test stop message confirm message create")

    def test_init_msg_create(self):
        self.assertEqual(mc.init_msg_create(1, 1.0).message_type, mt.MsgType.INIT, "test init message create")

    def test_init_msg_confirm_create(self):
        self.assertEqual(mc.init_msg_confirm_create(1, 1, 1.0).message_type, mt.MsgType.INIT_CONFIRM,
                         "test init message confirm create")

    def test_stop_comm_msg_create(self):
        self.assertEqual(mc.stop_comm_msg_create(1, 1.0).message_type, mt.MsgType.STOP_COMM,
                         "test stop comm handler message")

    def test_mutex_request_create(self):
        self.assertEqual(mc.mutex_request_create(1, 1, 1, 1.0).message_type, mt.MsgType.MUTEX_REQUEST,
                         "test mutex request message")

    def test_mutex_grant_create(self):
        self.assertEqual(mc.mutex_grant_create(1, 1, 1, 1, 1.0).message_type, mt.MsgType.MUTEX_GRANT,
                         "test mutex grant message")

    def test_mutex_release_create(self):
        self.assertEqual(mc.mutex_release_create(1, 1, 1.0).message_type, mt.MsgType.MUTEX_RELEASE,
                         "test mutex release message")


if __name__ == '__main__':
    unittest.main()