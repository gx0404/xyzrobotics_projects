# -*- coding: utf-8 -*-
"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Michael Su <michael.su@xyzrobotics.ai>, 2022, July, 2
"""

import unittest
import os 
import sys
import requests

from apps import create_app
from wcs_adaptor.app import init_app

FILE_PATH = os.path.abspath(__file__)
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(FILE_PATH)))))

class TestSendOrderLog(unittest.TestCase):

    def test_send_order_log(self):
        from utils.utils import send_order_log
        # Send legal message
        send_order_log(message="Success", status=True)
        send_order_log(message="Fail", status=False)


class TestSocketEvent(unittest.TestCase):

    def setUp(self):
        self.app = create_app(testing=True)
        init_app(self.app)
        self.client = self.app.test_client()
        self.socketio_client = self.app.socketio.test_client(
            self.app,
            flask_test_client=self.client
        )
        self.url_websocket = "/api/notify/websocket"

    def test_send_socket_event(self):
        """测试推送自定义socket事件"""
        # http请求
        event_name = "test_event"
        data = {
            "event": event_name,
            "data": {
                "message": "message"
            }
        }

        with self.app.app_context(), self.app.test_request_context():
            response = self.client.post(self.url_websocket, json=data)
            res_data = response.json
            messages = self.socketio_client.get_received("/")
            for msg in messages:
                if msg["name"] == event_name:
                    break
            else:
                raise Exception(f"接收socket服务推送的{event_name}事件失败！接收的数据：{messages}")
            assert res_data["code"] == 0, res_data

if __name__ == '__main__':
    unittest.main()