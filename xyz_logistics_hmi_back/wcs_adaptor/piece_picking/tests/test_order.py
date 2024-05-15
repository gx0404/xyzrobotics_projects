#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    tests.test_task_api
    ~~~~~~~~~~~~~~~~~~~~~

    piece picking中与订单、任务相关接口的单元测试脚本

    Copyright (c) XYZ Robotics Inc. - All Rights Reserved
    Unauthorized copying of this file, via any medium is strictly prohibited
    Proprietary and confidential
    Developer: Chen Kun <kun.chen@xyzrobotics.ai>, May, 2022
"""
import unittest
from apps import create_app
from wcs_adaptor.app import init_app
from wcs_adaptor.manager import task_manager, workspace_manager
from wcs_adaptor.piece_picking.wcs import standard_response as wcs_std_res

app = create_app(testing=True)
init_app(app)
client = app.test_client()
socketio_client = app.socketio.test_client(app, flask_test_client=client)


class TestOrder(unittest.TestCase):

    def create_order(self):
        """下发一个订单到HMI后端
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
        socketio_client.get_received("/")
        assert response.json in (
            wcs_std_res(
                code=-1,
                message="订单号{}已经存在!".format(data["order_id"])
            ),
            wcs_std_res()
        )

    def start_picking_service(self):
        pass

    def notice_picking_workspace_in(self):
        pass

    def notice_placing_workspace_in(self):
        pass

    def pick_n_place(self):
        pass

    def test_order(self):
        """
        """
        # 创建一个订单
        self.create_order()
        # 开启分拣服务
        self.start_picking_service()
        # 通知抓取空间就位
        self.notice_picking_workspace_in()
        # 通知放置空间就位
        self.notice_placing_workspace_in()
        # XTF请求获取订单信息
        self.pick_n_place()
