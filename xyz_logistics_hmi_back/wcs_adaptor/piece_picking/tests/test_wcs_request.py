#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    wcs_adaptor.piece_picking.tests.test_wcs_api.test_xtf_api
    ~~~~~~~~~~~~~~~~~~~~~

    piece picking中与xtf相关接口的单元测试脚本

    Copyright (c) XYZ Robotics Inc. - All Rights Reserved
    Unauthorized copying of this file, via any medium is strictly prohibited
    Proprietary and confidential
    Developer: Chen Kun <kun.chen@xyzrobotics.ai>, May, 2022
"""
import unittest

import requests

from apps import create_app
from wcs_adaptor.app import init_app
from wcs_adaptor.piece_picking.wcs import standard_response as wcs_std_res
from wcs_adaptor.piece_picking.wcs_request import (
    notice_system_is_ready,
    feedback_single_result,
    feedback_task_result,
    feedback_order_result
)

app = create_app(testing=True)
init_app(app)
client = app.test_client()
socketio_client = app.socketio.test_client(app, flask_test_client=client)


class TestWcsRequest(unittest.TestCase):

    def setUp(self):
        """测试接收WCS下发的订单信息
        """
        path = "/api/piece_picking/wcs/order"

        # 处理正常响应的情况
        # -----------------------------
        data = {
            "order_id": "0001",
            "tasks": [
                {
                    "picking_position": "1",
                    "picking_bin": "0",
                    "placing_position": "3",
                    "placing_bin": "0",
                    "sku_info": {
                        "sku_id": "001",
                        "length": 100.00,
                        "width": 100.00,
                        "height": 100.00,
                        "weight": 200.00,
                        "sku_type": "box"
                    },
                    "target_num": 3
                }
            ]
        }
        response = client.post(path, json=data)
        print(f'[+] ({path})响应的JSON数据: ', response and response.json)
        assert response.json in (
            wcs_std_res(
                code=-1,
                message="订单号{}已经存在!".format(data["order_id"])
            ),
            wcs_std_res()
        )

    def test_notice_system_is_ready(self):
        """通知WCS本系统分拣服务已准备就绪。"""
        response = notice_system_is_ready()
        print('[+] response json data: ', response and response.json)
        assert response is None or isinstance(response, requests.Response)

    def test_feedback_single_result(self):
        """向WCS反馈单次分拣结果。"""
        response = feedback_single_result()
        print('[+] response json data: ', response and response.json)
        assert response is None or isinstance(response, requests.Response)

    def test_feedback_task_result(self):
        """向WCS反馈任务结果。"""
        response = feedback_task_result()
        print('[+] response json data: ', response and response.json)
        assert response is None or isinstance(response, requests.Response)

    def test_feedback_order_result(self):
        """向WCS反馈订单结果。"""
        response = feedback_order_result()
        print('[+] response json data: ', response and response.json)
        assert response is None or isinstance(response, requests.Response)
