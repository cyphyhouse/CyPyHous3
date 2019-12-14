# Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

import unittest

import src.functionality.abstract.mutexHandler as mh


class DummyHandler(mh.MutexHandler):

    def __init__(self):
        super(DummyHandler, self).__init__()

    def grant_available_mutexes(self):
        pass

    def run(self):
        pass


class TestMutexHandler(unittest.TestCase):
    """
    testing the obstacle abstract type member access methods
    """

    def setUp(self):
        self.handler = DummyHandler()

    def test_stop(self):
        self.assertFalse(self.handler.stopped(), "testing stop event")
        self.handler.stop()
        self.assertTrue(self.handler.stopped(), "testing stop event")


if __name__ == '__main__':
    unittest.main()
