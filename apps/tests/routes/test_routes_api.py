# -*- coding: utf-8 -*-
"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022-07-20
"""
import unittest

from flask import url_for

from apps import create_app


class TestRoutesAPI(unittest.TestCase):

    def setUp(self) -> None:
        self.app = create_app(testing=True)
        self.client = self.app.test_client()

    def test_get_routes(self):
        """测试获取路由表."""
        with self.app.app_context(), self.app.test_request_context():
            resp = self.client.get(
                url_for("routes.list_routes", project_type="dpt")
            )
            data = resp.json["data"]
            assert data

    def test_update_route(self):
        """更新一个路由的名称."""
        with self.app.app_context(), self.app.test_request_context():
            js = {
                "route_id": 1,
                "name": "new_name"
            }
            resp = self.client.post(url_for("routes.edit_routes"), json=js)
            assert resp.json["data"]["name"] == "new_name", resp.json
