# Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

import unittest

from src.config.configs import AgentConfig


class TestAgentConfig(unittest.TestCase):
    """
    testing the agent config member access methods
    __moat_class : motion automaton specification
    """

    def setUp(self):
        self.ac = AgentConfig(0, 1, "127.1.1.1", 2000, [2000], None, True, None, None)

    def test_pid(self):
        self.assertEqual(self.ac.pid, 0, "pid test")

    def test_rip(self):
        self.assertEqual(self.ac.rip, "127.1.1.1", "receiver ip test")

    def test_rport(self):
        self.assertEqual(self.ac.rport, 2000, "listener port test")

    def test_plist(self):
        self.assertEqual(self.ac.plist, [2000], "listener ip list test")

    def test_mh(self):
        self.assertEqual(self.ac.mutex_handler, None, "mutex handler test")

    def test_is_leader(self):
        self.assertEqual(self.ac.is_leader, True, "leader test")

    def test_moat_class(self):
        self.assertEqual(self.ac.moat_class, None, "motion automaton test")

    def test_eq(self):
        self.assertEqual(self.ac, AgentConfig(0, 1, "127.1.1.1", 2000, [2000], None, True, None, None), "equality test")


if __name__ == '__main__':
    unittest.main()
