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
from apps import create_app
from wcs_adaptor.app import init_app
from wcs_adaptor.manager import (
    order_manager,
    task_manager,
    workspace_manager
)
from wcs_adaptor.piece_picking.entity import PPTask
from wcs_adaptor.piece_picking.wcs import standard_response as wcs_std_res
from wcs_adaptor.settings import wcs_settings


class TestXtfApi(unittest.TestCase):

    def setUp(self) -> None:
        """设置
        """
        from apps import settings
        settings.PROJECT_TYPE = "pp"
        self.app = create_app(testing=True)
        init_app(self.app)
        self.client = self.app.test_client()
        self.socketio_client = self.app.socketio.test_client(
            self.app,
            flask_test_client=self.client
        )
        self.url_prefix = "/api/piece_picking/xtf"
        # 清空之前的订单数据
        order_manager.clear()
        task_manager.clear()
        workspace_manager.clear()
    
    def tearDown(self) -> None:
        return super().tearDown()

    def test_get_task_info(self):
        """测试get_task_info接口的异常情况"""
        path = f"{self.url_prefix}/get_task_info"
        # 获取任务信息，获取成功result为True，任务信息获取失败result为False
        # 如果系统内部出现异常，则error返回1，如果正常则返回0

        # 测试正常响应的情况
        # -----------------------------
        response = self.client.post(path)
        print(f'[+] ({path})响应的JSON数据: ', response.json)
        assert response.json["error"] == 0

    def test_allow_pick(self):
        """测试allow_pick接口
        """
        path = f"{self.url_prefix}/allow_pick"

        data = {
            "task_id": "0001_0",
            "ws_id": "0_0"
        }

        # 测试正常响应的情况
        # -----------------------------
        response = self.client.post(path, json=data)
        print(f'[+] ({path})响应的JSON数据: ', response.json)
        assert response.json["error"] == 0

    def test_allow_move(self):
        """测试allow_move接口"""
        path = f"{self.url_prefix}/allow_move"

        data = {
            "task_id": "0001_0"
        }

        # 测试正常响应的情况
        # -----------------------------
        response = self.client.post(path, json=data)
        print(f'[+] ({path})响应的JSON数据: ', response.json)
        assert response.json["error"] == 0

    def test_allow_release(self):
        """测试allow_release接口"""
        path = f"{self.url_prefix}/allow_release"

        data = {
            "task_id": "0001_0"
        }

        # 测试正常响应的情况
        # -----------------------------
        response = self.client.post(path, json=data)
        print(f'[+] ({path})响应的JSON数据: ', response.json)
        assert response.json["error"] == 0

    def test_report_step_outcome(self):
        """测试report_step_outcome接口"""
        path = f"{self.url_prefix}/report_step_outcome"

        data = {
            "error": 0,
            "error_message": "",
            "task_id": "0001_0",
        }

        # 测试没有匹配的任务号的响应情况
        # -----------------------------
        response = self.client.post(path, json=data)
        print(f'[+] ({path})响应的JSON数据: ', response.json)
        assert response.json["error"] == 1

        # 测试有匹配的任务号的响应情况
        # -----------------------------
        # 下发订单
        order_data = {
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
        response = self.client.post("/api/piece_picking/wcs/order", json=order_data)
        print(f'[+] ({path})响应的JSON数据: ', response.json)
        assert response.json == wcs_std_res()
        # 获取任务
        response = self.client.post(f"{self.url_prefix}/get_task_info")
        print(f'[+] ({path})响应的JSON数据: ', response.json)
        assert response.json["error"] == 0
        # 反馈任务结果
        response = self.client.post(path, json=data)
        print(f'[+] ({path})响应的JSON数据: ', response.json)
        assert response.json["error"] == 0

    def test_error_handle(self):
        """测试handle_error接口，反馈xtf异常
        """
        path = f"{self.url_prefix}/error_handle"

        data = {
            "error": "0",
            "data": {
                "error_msg": "Start task failure",
                "zh_msg": "任务开始失败",
                "error_code": "E0000",
                "tip": "请联系XYZ开发人员"
            }
        }

        # 测试正常响应的情况
        # -----------------------------
        response = self.client.post(path, json=data)
        print(f'[+] ({path})响应的JSON数据: ', response.json)
        messages = self.socketio_client.get_received("/")
        assert response.json["error"] == 0

        # 测试异常响应情况
        # -----------------------------
        response = self.client.post(path, json={})
        print(f'[+] ({path})响应的JSON数据: ', response.json)
        assert response.json["error"] == 1
    
    def test_switch_tote(self):
        """测试换箱模式
        """
        path = f"{self.url_prefix}/report_step_outcome"

        data = {
            "error": 0,
            "error_message": "",
            "task_id": "0001_0",
        }

        # 测试有匹配的任务号的响应情况
        # -----------------------------
        
        # 设置工作空间转换表
        wcs_settings.pp_workspace_conversion = {
            '0_0': '0',
            '1_0': '1',
            '2_0': '2',
            '3_0': '3'
        }
        # 下发订单
        order_data = {
            "order_id": "0001",
            "tasks": [
                {
                    "task_type": 101,
                    "picking_position": "1",
                    "picking_bin": "0",
                    "placing_position": "2",
                    "placing_bin": "0",
                    "sku_info": {
                        "sku_id": "001",
                        "length": 100.00,
                        "width": 100.00,
                        "height": 100.00,
                        "weight": 200.00,
                        "sku_type": "box"
                    },
                    "target_num": 2
                }
            ]
        }
        response = self.client.post("/api/piece_picking/wcs/order", json=order_data)
        assert response.json == wcs_std_res()
        # 获取任务
        response = self.client.post(f"{self.url_prefix}/get_task_info")
        print(f'[+] ({path})响应的JSON数据: ', response.json)
        assert response.json["error"] == 0
        for i in range(order_data['tasks'][0]["target_num"] + 1):
            # 反馈任务结果
            response = self.client.post(path, json=data)
            print(f'[+] ({path})响应的JSON数据: ', response.json)
            assert response.json["error"] == 0
        # 检查任务数量是否有改变
        task = task_manager.first()
        assert task.from_ws() == "2" and task.to_ws() == "1"
    
    def test_disable_pp_xtf_api(self):
        """测试禁用piece picking的xtf相关接口"""
        wcs_settings.pp_xtf_api_map = {
            "get_task_info": False,
            "allow_pick": False,
            "allow_move": False,
            "allow_release": False,
            "report_step_outcome": False
        }
        
        data = {
            "task_id": "0001_0"
        }

        report_step_outcome_data = {
            "error": 0,
            "error_message": "",
            "task_id": "0001_0",
        }

        for key in wcs_settings.pp_xtf_api_map.keys():
            path = f"{self.url_prefix}/{key}"
            
            # 测试接口是否被禁用
            # -----------------------------
            if key == "get_task_info":
                response = self.client.post(path)
            elif key == "report_step_outcome":
                response = self.client.post(path, json=report_step_outcome_data)
            else:
                response = self.client.post(path, json=data)
            assert response.json["error"] == 0, response.json
            assert response.json["result"] == False, response.json
        
        wcs_settings.pp_xtf_api_map = {
            "get_task_info": True,
            "allow_pick": True,
            "allow_move": True,
            "allow_release": True,
            "report_step_outcome": True,
            "error_handle": True,
            "error": True,
        }