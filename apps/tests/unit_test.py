# -*- coding: utf-8 -*-
"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Michael Su <michael.su@xyzrobotics.ai>, 4/3/2022
"""
import os
import sys
import unittest

# Enable to import apps
FILE_PATH = os.path.abspath(__file__)
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(FILE_PATH))))

from local_helper.test_local_helper import TestLocalHelperPipeline
from system.test_system_api import TestSystemApi
from test_command import TestCommand
from dpt.test_map_box import TestMapBox
from dpt.test_design_api import TestDesignAPI
from dpt.design_v2 import TestPlanAPI, TestBoxAPI, TestPalletAPI
from test_error_handlers import TestErrorHandler
from customized_hmi.test_customized_hmi_api import TestCustomizedHmiApi
from notify.test_notify_api import TestSendOrderLog, TestSocketEvent
from test_utils import TestUtils
from routes.test_routes_api import TestRoutesAPI
from test_helpers import TestHelpers


def add_test_from_testcase(cls):
    """从测试用例类添加测试方法

    Args:
        cls: 测试用例类

    Returns:

    """
    tests = []
    for method in cls.__dict__:
        if not method.startswith("test_"):
            continue
        tests.append(cls(method))
    return tests


def suite():
    suites = unittest.TestSuite()
    suites.addTests(add_test_from_testcase(TestLocalHelperPipeline))
    suites.addTests(add_test_from_testcase(TestCommand))
    suites.addTests(add_test_from_testcase(TestMapBox))
    suites.addTests(add_test_from_testcase(TestDesignAPI))
    suites.addTests(add_test_from_testcase(TestSystemApi))
    suites.addTests(add_test_from_testcase(TestErrorHandler))
    suites.addTests(add_test_from_testcase(TestCustomizedHmiApi))
    suites.addTests(add_test_from_testcase(TestUtils))
    suites.addTests(add_test_from_testcase(TestSendOrderLog))
    suites.addTests(add_test_from_testcase(TestSocketEvent))
    suites.addTests(add_test_from_testcase(TestRoutesAPI))
    suites.addTests(add_test_from_testcase(TestPalletAPI))
    suites.addTests(add_test_from_testcase(TestBoxAPI))
    suites.addTests(add_test_from_testcase(TestPlanAPI))
    suites.addTests(add_test_from_testcase(TestHelpers))

    return suites


if __name__ == "__main__":
    runner = unittest.TextTestRunner()
    result = runner.run(suite())
    sys.exit(not result.wasSuccessful())
