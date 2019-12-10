# Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

# base test suite

import unittest

import src.tests.base.test_agent_config as test_agent_config
import src.tests.base.test_cylobs as test_cylobs
import src.tests.base.test_dsm as test_dsm
import src.tests.base.test_gvh as test_gvh
import src.tests.base.test_message as test_message
import src.tests.base.test_moat_config as test_moat_config
import src.tests.base.test_mutex_handler as test_mutex_handler
import src.tests.base.test_obstacle as test_obstacle
import src.tests.base.test_planner as test_planner
import src.tests.base.test_pos as test_pos
import src.tests.base.test_roundobs as test_roundobs
import src.tests.base.test_seg as test_seg
import src.tests.base.test_simple_planner as test_simple_planner
import src.tests.base.test_msg_create as test_message_create


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
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    return result


if __name__ == "__main__":
    main()
