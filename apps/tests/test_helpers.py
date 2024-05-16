#  -*- coding: utf-8 -*-
#  Copyright (c) XYZ Robotics Inc. - All Rights Reserved
#  Unauthorized copying of this file, via any medium is strictly prohibited
#  Proprietary and confidential
#  Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022/8/10 下午2:21
import json
import unittest

from apps import create_app
from apps.helpers import record_error
from apps.base_app.views.error_records.crud import crud_error_records


class Task(object):

    def __init__(self):
        self.data = {
            "task_id": 1,
            "order_id": 1,
        }

    def dict(self) -> dict:
        return self.data

    def json(self) -> str:
        return json.dumps(self.data)


class TestHelpers(unittest.TestCase):
    def setUp(self) -> None:
        self.app = create_app(testing=True)

    def test_record_error(self):
        task = Task()
        with self.app.app_context():
            # 记录XTF的异常
            error_record = record_error(code="1000", source="xtf", task=task)
            result = crud_error_records.get(pk=error_record.id)
            assert result is not None
