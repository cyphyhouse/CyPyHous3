# Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

# basic test suite

import unittest

import src.tests.basic.test_agent_config as test_agent_config
import src.tests.basic.test_cylobs as test_cylobs
import src.tests.basic.test_dsm as test_dsm
import src.tests.basic.test_get_configs as test_get_configs
import src.tests.basic.test_gvh as test_gvh
import src.tests.basic.test_message as test_message
import src.tests.basic.test_moat_config as test_moat_config
import src.tests.basic.test_msg_create as test_message_create
import src.tests.basic.test_mutex_handler as test_mutex_handler
import src.tests.basic.test_obstacle as test_obstacle
import src.tests.basic.test_planner as test_planner
import src.tests.basic.test_pos as test_pos
import src.tests.basic.test_roundobs as test_roundobs
import src.tests.basic.test_seg as test_seg
import src.tests.basic.test_simple_planner as test_simple_planner
import src.tests.basic.test_basicMutex as test_basicMutex


# noinspection PyTypeChecker
def main():
    suite = unittest.TestSuite()
    loader = unittest.TestLoader()
    suite.addTest(loader.loadTestsFromModule(test_agent_config))
    suite.addTest(loader.loadTestsFromModule(test_moat_config))
    suite.addTest(loader.loadTestsFromModule(test_mutex_handler))
    suite.addTest(loader.loadTestsFromModule(test_obstacle))
    suite.addTest(loader.loadTestsFromModule(test_planner))
    suite.addTest(loader.loadTestsFromModule(test_pos))
    suite.addTest(loader.loadTestsFromModule(test_roundobs))
    suite.addTest(loader.loadTestsFromModule(test_seg))
    suite.addTest(loader.loadTestsFromModule(test_cylobs))
    suite.addTest(loader.loadTestsFromModule(test_message))
    suite.addTest(loader.loadTestsFromModule(test_simple_planner))
    suite.addTest(loader.loadTestsFromModule(test_dsm))
    suite.addTest(loader.loadTestsFromModule(test_gvh))
    suite.addTest(loader.loadTestsFromModule(test_message_create))
    suite.addTest(loader.loadTestsFromModule(test_get_configs))
    suite.addTest(loader.loadTestsFromModule(test_basicMutex))

    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    return result


if __name__ == "__main__":
    main()
