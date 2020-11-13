# Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

# base test suite

import unittest

import tests.base.test_agent_config as test_agent_config
import tests.base.test_agent_thread as test_agent_thread
import tests.base.test_cylobs as test_cylobs
import tests.base.test_message as test_message
import tests.base.test_moat_config as test_moat_config
import tests.base.test_mutex_handler as test_mutex_handler
import tests.base.test_obstacle as test_obstacle
import tests.base.test_planner as test_planner
import tests.base.test_pos as test_pos
import tests.base.test_roundobs as test_roundobs
import tests.base.test_seg as test_seg
import tests.base.test_msg_create as test_msg_create
import tests.base.test_dsm as test_dsm
import tests.base.test_dsm_allread as test_dsm_allread
import tests.base.test_dsm_allwrite as test_dsm_allwrite
import tests.base.test_gvh as test_gvh


def main():
    suite = unittest.TestSuite()
    loader = unittest.TestLoader()
    modules = [
        test_agent_config,
        test_agent_thread,
        test_moat_config,
        test_mutex_handler,
        test_obstacle,
        test_planner,
        test_pos,
        test_roundobs,
        test_seg,
        test_cylobs,
        test_message,
        test_msg_create,
        test_dsm,
        test_dsm_allread,
        test_dsm_allwrite,
        test_gvh
    ]
    suite.addTests(loader.loadTestsFromModule(m) for m in modules)
    runner = unittest.TextTestRunner(verbosity=3)
    result = runner.run(suite)
    return result


if __name__ == "__main__":
    main()
