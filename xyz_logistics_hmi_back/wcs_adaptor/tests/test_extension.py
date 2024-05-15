#  -*- coding: utf-8 -*-
#  -*- coding: utf-8 -*-
#  Copyright (c) XYZ Robotics Inc. - All Rights Reserved
#  Unauthorized copying of this file, via any medium is strictly prohibited
#  Proprietary and confidential
#  Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022/8/12 上午10:17
import unittest
import uuid

from flask import url_for
from apps import create_app

import wcs_adaptor
from wcs_adaptor.manager import task_manager
from wcs_adaptor.helpers import backup_manager


class TestExtension(unittest.TestCase):
    def setUp(self) -> None:
        self.app = create_app()
        self.client = self.app.test_client()
        wcs_adaptor.init_app(self.app)

    def tearDown(self) -> None:
        with self.app.app_context():
            task_manager.clear()
            backup_manager()

    def test_get_current_task(self):
        """测试获取当前任务."""
        task_manager.clear()
        with self.app.app_context(), self.app.test_request_context():
            # 创建单拆任务
            task_id = uuid.uuid4().__str__()
            js = {
                "task_id": task_id,
                "sku_info": {
                    "sku_id": "",
                    "length": 1000,
                    "width": 1000,
                    "height": 1000,
                    "weight": 5,
                },
                "from": "1",
                "to": "2",
                "target_num": 1,
            }
            resp = self.client.post(url_for("wcs.single_class_depal_task"), json=js)
            assert resp.json["code"] == 0, resp.json

            # 获取当前任务
            resp = self.client.get(url_for("ext.get_current_task"))
            assert resp.json["code"] == 0, resp.json
            assert resp.json["data"]["task_id"] == task_id, resp.json
        task_manager.clear()
