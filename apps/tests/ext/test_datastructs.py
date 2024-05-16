#  -*- coding: utf-8 -*-
#  Copyright (c) XYZ Robotics Inc. - All Rights Reserved
#  Unauthorized copying of this file, via any medium is strictly prohibited
#  Proprietary and confidential
#  Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022/9/20 上午10:08
import contextlib
import unittest

import pydantic
from pydantic import Field

from apps.ext.datastructs import ABCLiteTask


class TestTask(unittest.TestCase):
    """测试任务基类"""

    def test_inherit(self):
        """测试继承."""

        class DPTTask(ABCLiteTask):
            def start(self):
                pass

            def finish(self):
                pass

            def terminate(self):
                pass

            def report_pick_num(self, num: int, **kwargs):
                self.done_num += num

            from_ws: str = Field(description="来自哪个工作站")
            to_ws: str = Field(description="去哪个工作站")

        task = DPTTask(task_id="123", target_num=1, from_ws="A", to_ws="B")
        assert task.task_id == "123"

        # 无法修改
        with contextlib.suppress(TypeError):
            task.done_num = 2
        assert task.done_num == 0

        # 调用方法可以修改
        task.report_pick_num(1)
        assert task.done_num == 1, task.done_num

        # 测试多重继承
        class DPTTask2(DPTTask):
            a: int = 0
            b: int = 1

        task2 = DPTTask2(task_id="456", target_num=1, from_ws="A", to_ws="B")
        assert task2.a == 0
        assert task2.b == 1

        with contextlib.suppress(TypeError):
            task2.done_num = 1
        task2.report_pick_num(1)
        assert task2.done_num == 1, task2.done_num
        assert task2.task_id == "456"

        # 测试参数完整性
        task3 = None
        with contextlib.suppress(pydantic.error_wrappers.ValidationError):
            task3 = DPTTask2(task_id="456", target_num=1, from_ws="A")
        assert task3 is None
