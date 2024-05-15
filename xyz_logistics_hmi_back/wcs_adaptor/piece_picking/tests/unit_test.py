#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    tests.unit_test
    ~~~~~~~~~~~~~~~~~~~~~

    piece picking的单元测试脚本测试入口

    Copyright (c) XYZ Robotics Inc. - All Rights Reserved
    Unauthorized copying of this file, via any medium is strictly prohibited
    Proprietary and confidential
    Developer: Chen Kun <kun.chen@xyzrobotics.ai>, May, 2022
"""
import os
import sys
import unittest

from apps.settings import settings

basedir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, basedir)

# 添加分拣站使用的特定参数'-pp'
# sys.argv.append('-pp')
settings.PROJECT_TYPE = "pp"

# 因为wcs_adaptor与apps相对目录的问题,
# 如需导入apps, 则需主动将其apps加入到sys.path中
from wcs_adaptor.piece_picking.tests.test_wcs_api import TestWcsApi
from wcs_adaptor.piece_picking.tests.test_xtf_api import TestXtfApi
from wcs_adaptor.piece_picking.tests.test_wcs_request import TestWcsRequest
from wcs_adaptor.piece_picking.tests.test_order import TestOrder


def suite():
    suites = unittest.TestSuite()
    suites.addTests(TestWcsApi)
    suites.addTests(TestXtfApi)
    suites.addTests(TestWcsRequest)
    suites.addTests(TestOrder)
    return suites


if __name__ == '__main__':
    runner = unittest.TextTestRunner()
    result = runner.run(suite())
    sys.exit(not result.wasSuccessful())
