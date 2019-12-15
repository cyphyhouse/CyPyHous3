#  Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

from unittest import TestCase, main

from src.objects.basicMutex import BasicMutex


class TestBasicMutex(TestCase):
    def setUp(self):
        self.bmx = BasicMutex("test")
        pass

    def test_mutex_id(self):
        self.assertEqual(self.bmx.mutex_id, "test", "Testing mutex id")

    def test_mutex_request_list(self):
        self.assertEqual(self.bmx.mutex_request_list, [], "Testing mutex request list")
        self.bmx.mutex_request_list.append(1)
        self.assertEqual(self.bmx.mutex_request_list[0], 1, "Testing mutex request list")

    def test_mutex_holder(self):
        self.assertEqual(self.bmx.mutex_holder, None, "Testing mutex holder")
        self.bmx.mutex_holder = 1
        self.assertEqual(self.bmx.mutex_holder, 1, "Testing mutex holder")

    def test_agent_comm_handler(self):
        self.assertEqual(self.bmx.agent_comm_handler, None, "Testing comm handler")
        pass


if __name__ == '__main__':
    main()
