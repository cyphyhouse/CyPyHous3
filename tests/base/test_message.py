# Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

import unittest

from src.objects.message import Message


class TestMessage(unittest.TestCase):
    """
    testing the message type member access methods

    """

    def setUp(self):
        self.msg = Message(1, 2, "3", 4.0)

    def test_sender(self):
        self.assertEqual(self.msg.sender, 1, "test message sender")

    def test_type(self):
        self.assertEqual(self.msg.message_type, 2, "test message type")

    def test_content(self):
        self.assertEqual(self.msg.content, "3", "test content")

    def test_time_stamp(self):
        self.assertEqual(self.msg.timestamp, 4.0, "test timestamp")

    def test_equality(self):
        self.assertEqual(self.msg, Message(1, 2, "3", 4.0), "testing message equality")


if __name__ == '__main__':
    unittest.main()
