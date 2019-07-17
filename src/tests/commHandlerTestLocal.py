"""Test CommHandler on localhost"""

import time
import unittest

from src.sender import Sender
from src.receiver import Receiver
from src.message import Message


class TestSenderReceiver(unittest.TestCase):
    def setUp(self) -> None:
        self.sender = Sender(ip='127.0.0.1', port=2005)
        self.receiver = Receiver(ip='127.0.0.1', port=2006)

    def test_send_recv(self):
        msg = Message(m_type=0,
                      sender=0,
                      content="hello",
                      ts=1.0)

        self.sender.send(msg)
        time.sleep(1)
        ret_msg = self.receiver.recv()

        unittest.AssertEqual(str(msg), str(ret_msg))

"""
class TestCommHandler(unittest.TestCase):

    def setUp(self) -> None:
        self.sender = CommHandler('127.0.0.1', 2003, 2004, 0)
        self.receiver = CommHandler('127.0.0.1', 2004, 2003, 1)

    def test_send(self) -> None:

        msg = Message(m_type=0,
                      sender=self.sender.pid,
                      content="hello",
                      ts=1.0)
        self.sender.send(msg)
        self.receiver.run()

        unittest.AssertEqual()
"""

if __name__ == '__main__':
    unittest.main()
