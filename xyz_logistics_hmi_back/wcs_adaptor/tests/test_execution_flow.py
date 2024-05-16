# -*- coding: utf-8 -*-
"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022-07-09

流程测试，用于测试完整的任务执行流程.
"""
import random
import time
import unittest
import uuid

import requests
from flask import url_for

from apps import create_app
from wcs_adaptor.manager import order_manager, task_manager, workspace_manager


class TestExecutionFlow(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(TestExecutionFlow, self).__init__(*args, **kwargs)
        self.app = create_app(testing=True)
        self.client = self.app.test_client()
        self.socketio = self.app.socketio
        self.socketio_client = self.socketio.test_client(
            self.app, flask_test_client=self.client
        )
        import wcs_adaptor

        wcs_adaptor.init_app(self.app)

    def clear_manager(self):
        """清空环境"""
        task_manager.clear()
        order_manager.clear()
        workspace_manager.clear()

    def setUp(self) -> None:
        self.clear_manager()

    def tearDown(self) -> None:
        self.clear_manager()

    def get_task_info(self) -> dict:
        """模拟XTF获取任务信息."""
        resp = self.client.post(url_for("rafcon.get_task_info"))
        assert resp.json["code"] == 0
        return resp.json["data"]

    def report_task_status(
        self,
        task_id: str,
        pick_num: int,
        drop_num: int = 0,
        is_depal_pallet_empty: bool = False,
        is_pal_pallet_full: bool = False,
        error: int = 0,
        error_message: str = "success",
    ):
        """模拟XTF回报任务状态."""
        resp = self.client.post(
            url_for("rafcon.report_task_status"),
            json={
                "task_id": task_id,
                "pick_num": pick_num,
                "drop_num": drop_num,
                "is_depal_pallet_empty": is_depal_pallet_empty,
                "is_pal_pallet_full": is_pal_pallet_full,
                "error": error,
                "error_message": error_message,
            },
        )
        assert resp.json["code"] == 0, resp.json

    def report_task_ending(
        self,
        task_id: str,
    ):
        """模拟XTF回报任务状态."""
        resp = self.client.post(
            url_for("rafcon.report_task_ending"),
            json={
                "task_id": task_id,
            },
        )
        assert resp.json["code"] == 0, resp.json

    def test_single_depal_task(self):
        """测试单拆任务流程."""
        pass

    def test_single_pal_task(self):
        """测试单码任务流程."""
        pass

    def test_multi_depal_task(self):
        """测试混拆任务流."""

        with self.app.app_context(), self.app.test_request_context():
            # 清空环境
            self.clear_manager()

            # 创建指定数量的混拆任务
            api = url_for("wcs.multi_class_depal_task")

            task_id = random.randint(1, 100).__str__()
            resp = self.client.post(
                api,
                json={
                    "task_id": task_id,
                    "from": "0",
                    "to": "1",
                    "target_num": 2,
                },
            )
            assert resp.json["code"] == 0

            # 获取任务
            rv = self.get_task_info()
            assert rv["result"] == True
            assert rv["task_id"] == task_id and rv["undone_num"] == 2

            # 报告一次抓取任务
            self.report_task_status(task_id=task_id, pick_num=1)
            rv = self.get_task_info()
            assert rv["undone_num"] == 1

            # 报告任务完成
            self.report_task_status(task_id=task_id, pick_num=1)
            rv = self.get_task_info()
            assert rv["result"] == True

            # 报告任务结束
            self.report_task_ending(task_id=task_id)
            rv = self.get_task_info()
            assert rv["result"] == False

            # 创建数量为-1的混拆任务
            task_id = random.randint(1, 100).__str__()
            self.client.post(
                api,
                json={
                    "task_id": task_id,
                    "from": "0",
                    "to": "1",
                    "target_num": -1,
                },
            )

            # 获取任务信息
            rv = self.get_task_info()
            assert rv["result"] == True and rv["task_id"] == task_id

            # 报告任务完成(拆跺托盘已空)
            self.report_task_status(
                task_id=task_id, pick_num=1, is_depal_pallet_empty=True
            )

            # 上报任务结束
            self.report_task_ending(task_id=task_id)

            # 创建数量为-1的混拆任务
            task_id = random.randint(1, 100).__str__()
            self.client.post(
                api,
                json={
                    "task_id": task_id,
                    "from": "0",
                    "to": "1",
                    "target_num": -1,
                },
            )
            # 获取任务信息
            rv = self.get_task_info()
            assert rv["result"] == True and rv["task_id"] == task_id

            # 报告任务完成(码跺托盘已满)
            self.report_task_status(
                task_id=task_id, pick_num=1, is_pal_pallet_full=True
            )

            self.report_task_ending(task_id=task_id)

            # 清空环境
            self.clear_manager()

    def test_multi_online_pal_task(self):
        """测试在线混码任务流."""
        with self.app.app_context(), self.app.test_request_context():
            # 创建一个混码任务
            def create_online_multipal_task(_id: str, _from: str, _to: str):
                data = {
                    "task_id": _id,
                    "sku_info": {
                        "sku_id": "",
                        "length": 1000,
                        "width": 1000,
                        "height": 1000,
                        "weight": 5,
                        "sku_num": 10,
                    },
                    "from": _from,
                    "to": _to,
                }
                self.client.post(url_for("wcs.multi_class_pal_task_online"), json=data)
                return task_manager.get_task_by_id(_id)

            """
            报告混码任务（放置托盘无异常）
            """
            self.clear_manager()  # 清空管理器
            task_id = "xyz001"
            from_ws = "0"
            to_ws = "1"
            task = create_online_multipal_task(task_id, from_ws, to_ws)  # 创建在线混码任务
            # 获取在线混吗任务
            resp = self.client.post(url_for("rafcon.get_task_info"))
            assert resp.json["code"] == 0 and resp.json["result"] == True, resp.json
            # 查询空间就位状态（未通知就位，状态应该是未就位）
            resp = self.client.post(
                url_for("rafcon.is_pick_ws_ready"),
                json={"task_id": task_id, "ws_id": from_ws},
            )
            assert resp.json["code"] == 0 and resp.json["result"] == False, resp.json
            resp = self.client.post(
                url_for("rafcon.is_place_ws_ready"),
                json={"task_id": task_id, "ws_id": to_ws},
            )
            assert resp.json["code"] == 0 and resp.json["result"] == False, resp.json
            # 通知抓取空间就位
            resp = self.client.post(
                url_for("wcs.notice_pick_ws_ready"),
                json={"task_id": task_id, "ws_id": from_ws, "pallet_id": ""},
            )
            assert resp.json["code"] == 0, resp.json
            # 通知放置空间就位
            resp = self.client.post(
                url_for("wcs.notice_pick_ws_ready"),
                json={"task_id": task_id, "ws_id": to_ws, "pallet_id": ""},
            )
            assert resp.json["code"] == 0, resp.json
            # 再次查询空间就位状态（通知就位后，状态已经已经就位了）
            resp = self.client.post(
                url_for("rafcon.is_pick_ws_ready"),
                json={"task_id": task_id, "ws_id": from_ws},
            )
            assert resp.json["code"] == 0 and resp.json["result"] == True, resp.json
            resp = self.client.post(
                url_for("rafcon.is_place_ws_ready"),
                json={"task_id": task_id, "ws_id": to_ws},
            )
            assert resp.json["code"] == 0 and resp.json["result"] == True, resp.json
            # 获取任务结果
            resp = self.client.post(
                url_for("rafcon.report_task_status"),
                json={
                    "task_id": task_id,
                    "pick_num": 3,
                    "error": 0,
                    "error_message": "",
                    "is_depal_pallet_empty": False,
                    "is_pal_pallet_full": False,
                },
            )
            assert resp.json["code"] == 0, resp.json
            # 查询工作空间就位状态，此处因为提交empty和full的都为False，所以result应该是True
            resp = self.client.post(
                url_for("rafcon.is_pick_ws_ready"),
                json={"task_id": task_id, "ws_id": from_ws},
            )
            assert resp.json["code"] == 0 and resp.json["result"] == True, resp.json
            resp = self.client.post(
                url_for("rafcon.is_place_ws_ready"),
                json={"task_id": task_id, "ws_id": to_ws},
            )
            assert resp.json["code"] == 0 and resp.json["result"] == True, resp.json

            """
            空、满状态的异常校验
            """
            # xtf报告任务结果，空异常
            resp = self.client.post(
                url_for("rafcon.report_task_status"),
                json={
                    "task_id": task_id,
                    "pick_num": 1,
                    "error": 0,
                    "error_message": "",
                    "is_depal_pallet_empty": True,
                    "is_pal_pallet_full": False,
                },
            )
            assert resp.json["code"] == 0, resp.json
            # 查询工作空间就位状态，此处应该得到result为False，因为空异常应该会导致抓取空间不可用
            resp = self.client.post(
                url_for("rafcon.is_pick_ws_ready"),
                json={"task_id": task_id, "ws_id": from_ws},
            )
            assert resp.json["code"] == 0 and resp.json["result"] == False, resp.json
            resp = self.client.post(
                url_for("wcs.notice_pick_ws_ready"),
                json={"task_id": task_id, "ws_id": from_ws, "pallet_id": ""},
            )
            assert resp.json["code"] == 0

            # xtf报告任务结果，满异常
            resp = self.client.post(
                url_for("rafcon.report_task_status"),
                json={
                    "task_id": task_id,
                    "pick_num": 1,
                    "error": 0,
                    "error_message": "",
                    "is_depal_pallet_empty": False,
                    "is_pal_pallet_full": True,
                },
            )
            assert resp.json["code"] == 0, resp.json
            # 查询工作空间就位状态，此处应该得到result为False，因为满异常应该会导致放置空间不可用
            resp = self.client.post(
                url_for("rafcon.is_place_ws_ready"),
                json={"task_id": task_id, "ws_id": to_ws},
            )
            assert resp.json["code"] == 0 and resp.json["result"] == False, resp.json
            resp = self.client.post(
                url_for("wcs.notice_place_ws_ready"),
                json={"task_id": task_id, "ws_id": to_ws, "pallet_id": ""},
            )
            assert resp.json["code"] == 0

            # 清空任务
            task_manager.clear()

    def test_multi_offline_pal_task(self):
        """测试离线混码任务流."""
        with self.app.app_context(), self.app.test_request_context():
            # 清空环境
            # self.clear_manager()
            order_id = uuid.uuid4().__str__()
            resp = requests.post(
                "http://127.0.0.1:7002" + url_for("wcs.multi_class_pal_task_offline"),
                json={
                    "order_id": order_id,
                    "from": "1",
                    "to": "0",
                    "sku_info": [
                        {
                            "weight": 5.0,
                            "length": 400,
                            "width": 350,
                            "height": 300,
                            "sku_id": "6904579670800",
                            "sku_num": 1,
                        },
                        {
                            "weight": 4.5,
                            "length": 600,
                            "width": 350,
                            "height": 300,
                            "sku_id": "6907179110814",
                            "sku_num": 2,
                        },
                    ],
                },
            )
            assert resp.json()["code"] == 0, resp.json()

            # 等待规划完成
            time.sleep(10)

            resp = requests.post(
                "http://127.0.0.1:7002"
                + url_for("wcs.multi_class_pal_task_offline_start"),
                json={"order_id": order_id, "round_id": 0},
            )
            assert resp.json()["code"] == 0, resp.json()

            task_id = order_id + "_0"
            # 回报任务结果
            resp = requests.post(
                "http://127.0.0.1:7002" + url_for("rafcon.report_task_status"),
                json={
                    "task_id": task_id,
                    "pick_num": 1,
                    "is_depal_pallet_empty": True,
                    "is_pal_pallet_full": True,
                    "error": 0,
                    "error_message": "success",
                },
            )
            assert resp.json()["code"] == 0, resp.json()
