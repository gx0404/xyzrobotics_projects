#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    wcs_adaptor.piece_picking.tests.test_wcs_api
    ~~~~~~~~~~~~~~~~~~~~~

    piece picking中与wcs相关接口的单元测试脚本

    Copyright (c) XYZ Robotics Inc. - All Rights Reserved
    Unauthorized copying of this file, via any medium is strictly prohibited
    Proprietary and confidential
    Developer: Chen Kun <kun.chen@xyzrobotics.ai>, May, 2022
"""
import unittest
from apps import create_app
from wcs_adaptor.app import init_app
from wcs_adaptor.piece_picking.wcs import standard_response

app = create_app(testing=True)
init_app(app)
socketio = app.socketio
client = app.test_client()
socketio_client = socketio.test_client(app, flask_test_client=client)


class TestWcsApi(unittest.TestCase):

    def test_receive_order(self):
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
        socketio_client.get_received("/")
        assert response.json in (
            standard_response(
                code=-1,
                message="订单号{}已经存在!".format(data["order_id"])
            ),
            standard_response()
        )

    def test_notice_ws_ready(self):
        """测试工作空间已就位
        """
        path = "/api/piece_picking/wcs/notice_ws_ready"

        # 处理正常响应的情况
        # -----------------------------
        data = {
            "ws_id": "1"
        }
        response = client.post(path, json=data)
        assert response.json == standard_response()

    def test_notice_ws_release(self):
        """测试工作空间未就位
        """
        path = "/api/piece_picking/wcs/notice_ws_release"

        # 处理正常响应的情况
        # -----------------------------
        data = {
            "ws_id": "1"
        }
        response = client.post(path, json=data)
        assert response.json == standard_response()

    def test_read_plc(self):
        """测试读取PLC信号（仅测试接口是否正常请求）
        """
        path = "/api/piece_picking/wcs/read_plc"

        # 处理正常响应的情况
        # -----------------------------
        data = {
            "address": 10,
            "length": 1
        }
        response = client.post(path, json=data)
        print(response)
        assert response.json["code"] in (0, -1)

    def test_write_plc(self):
        """测试写入PLC信号（仅测试接口是否正常请求）
        """
        path = "/api/piece_picking/wcs/write_plc"

        # 处理正常响应的情况
        # -----------------------------
        # 20号地址为改变三色灯的颜色成绿色
        data = {
            "address": 20,
            "value": [1, 0, 0]
        }
        response = client.post(path, json=data)
        print(response.json)
        assert response.json["code"] in (0, -1)
    
    def test_robot_motion(self):
        """测试写入PLC信号（仅测试接口是否正常请求）
        """
        path = "/api/piece_picking/wcs/robot_motion"
        
        # 测试接口是否被禁用
        # -----------------------------
        data = {
            "get_task_info": True,
        }
        response = client.post(path, json=data)
        assert response.json["code"] == 0, response.json

        data = {
            "get_task_info": False,
            "allow_pick": False,
            "allow_move": False,
            "allow_release": False,
            "report_step_outcome": False
        }
        response = client.post(path, json=data)
        assert response.json["code"] == 0, response.json

        data = {
            "get_task_info": True,
            "allow_pick": True,
            "allow_move": True,
            "allow_release": True,
            "report_step_outcome": True
        }
        response = client.post(path, json=data)
        assert response.json["code"] == 0, response.json
            
