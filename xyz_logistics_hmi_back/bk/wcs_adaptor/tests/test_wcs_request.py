# -*- coding: utf-8 -*-
"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022-05-18

测试wcs_request.
"""
import unittest

from apps import create_app
from wcs_adaptor.depalletize.wcs_request import (
    REPORT_ACTION_STATUS_API,
    send_request
)

app = create_app()
socketio = app.socketio
client = app.test_client()
socketio_client = socketio.test_client(app, flask_test_client=client)


class TestWCSRequest(unittest.TestCase):

    def test_send_request(self):
        """测试向WCS发送请求."""
        resp = send_request(REPORT_ACTION_STATUS_API, {}, 1)
        assert resp.status_code == 200, f"状态码错误, {resp.status_code}"

    def test_send_error_request(self):
        """测试WCS返回异常响应的情况."""
        # 请求一个不存在的路由
        resp = send_request("dddddddddddddd", {}, 3)
        assert resp is None
        # message = socketio_client.get_received("/")[-1]
        # assert message["args"][0]["status"] is False, message
