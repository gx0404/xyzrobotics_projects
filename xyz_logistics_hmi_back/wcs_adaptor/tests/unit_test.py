# -*- coding: utf-8 -*-
"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022-04-13
"""
import os
import sys
import unittest
from pathlib import Path

# 因为wcs_adaptor与apps相对目录的问题,
# 如需导入apps, 则需主动将其apps加入到sys.path中
from apps import settings, create_app

path = Path(os.path.realpath(__file__)).parent.__str__()
sys.path.insert(0, path)

create_app()

from test_task_api import TestWCSTask
from test_wcs_api import TestWCSApi
from test_rafcon_api import TestRafconApi
from test_wcs_request import TestWCSRequest
from test_order_api import TestOrder
from test_task_history_api import TestTaskHistoryAPI
from test_lowcode_api import TestLowcodeApi
from test_execution_flow import TestExecutionFlow
from test_extension import TestExtension

settings.PROJECT_TYPE = "dpt"


def add_test_from_testcase(cls):
    """从测试用例类添加测试方法

    Args:
        cls: 测试用例类

    Returns:

    """
    return [cls(method) for method in cls.__dict__ if
            method.startswith("test_")]


def suite():
    suites = unittest.TestSuite()
    suites.addTests(add_test_from_testcase(TestWCSTask))
    suites.addTests(add_test_from_testcase(TestWCSApi))
    suites.addTests(add_test_from_testcase(TestRafconApi))
    suites.addTests(add_test_from_testcase(TestWCSRequest))
    suites.addTests(add_test_from_testcase(TestOrder))
    suites.addTests(add_test_from_testcase(TestTaskHistoryAPI))
    suites.addTests(add_test_from_testcase(TestLowcodeApi))
    suites.addTests(add_test_from_testcase(TestExecutionFlow))
    suites.addTests(add_test_from_testcase(TestExtension))
    return suites


if __name__ == '__main__':
    runner = unittest.TextTestRunner()
    result = runner.run(suite())
    sys.exit(not result.wasSuccessful())
