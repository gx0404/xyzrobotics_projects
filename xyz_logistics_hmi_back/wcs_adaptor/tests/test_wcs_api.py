# -*- coding: utf-8 -*-
"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022-04-25
"""
import random
import string
import unittest
import uuid
from typing import List, Dict

from flask.wrappers import Response

from apps import create_app
from wcs_adaptor.app import init_app
from wcs_adaptor.enums import TaskType
from wcs_adaptor.helpers import calculate_sku_order
from wcs_adaptor.manager import task_manager, order_manager, workspace_manager

url_prefix = "/api/wcs"


class TestWCSApi(unittest.TestCase):
    
    def __init__(self, *args, **kwargs):
        self.app = create_app(testing=True)
        init_app(self.app)
        self.socketio = self.app.socketio
        self.client = self.app.test_client()
        # self.socketio_client = self.socketio.test_client(self.app, flask_test_client=self.client)
        super(TestWCSApi, self).__init__(*args, **kwargs)

    def tearDown(self) -> None:
        # self.socketio_client.get_received("/")
        task_manager.clear()

    def test_notice_pick_ws_ready_input_error(self):
        """测试通知夹取位是否就绪 - 错误输入
        
        Returns:

        """
        # 传递一个错误的请求体
        resp = self.client.post(url_prefix + "/notice_pick_ws_ready", json={})
        resp_json = resp.json
        assert resp_json["code"] == 10400, resp_json

    def test_notice_pick_ws_ready_success(self):
        """测试通知夹取位是否就绪 - 成功
        
        Returns:

        """

        # 创建一个单拆任务
        task_id = "".join(random.choices(string.digits, k=4))
        resp = self.create_task(task_id=task_id)
        assert resp.json == {
            'code': 0,
            'msg': 'success',
            'data': {},
            'error': 0,
            'error_message': 'success'
        }, resp.json
        # messages: List[Dict] = self.socketio_client.get_received("/")

        data = {"task_id": task_id, "ws_id": "0", "pallet_id": "1"}
        # 通知夹取位已到位
        resp = self.client.post(url_prefix + "/notice_pick_ws_ready", json=data)
        resp_json = resp.json
        assert resp_json == {
            'code': 0,
            'msg': 'success',
            'data': {},
            'error': 0,
            'error_message': 'success'
        }, resp_json

        # 判断任务管理器里对应的抓取空间状态是否为True
        assert workspace_manager.get("0").is_ready

        # 判断HMI是否收到推送消息
        # messages: List[Dict] = self.socketio_client.get_received("/")
        # assert messages[0]["name"] == "order_info" and messages[0]["args"][0][
        #     "msg"] == "夹取位0已到位"

        # 清空任务, 避免影响其他测试用例
        self.remove_task()

    def test_notice_place_ws_ready_input_error(self):
        """测试放置位是否就绪 - 输入异常
        
        Returns:

        """
        # 传递一个错误的请求体
        resp = self.client.post(url_prefix + "/notice_place_ws_ready", json={})
        resp_json = resp.json
        assert resp_json["code"] == 10400, resp_json
        # 校验消息推送
        # messages = self.socketio_client.get_received("/")
        # assert (messages[0]["name"] == "order_info" and
        #         messages[0]["args"][0]["msg"] == resp_json[
        #             "error_message"])

    def test_notice_place_ws_ready_success(self):
        """测试放置位是否就绪 - 成功
        
        Returns:

        """

        # 创建一个单拆任务
        task_id = "".join(random.choices(string.digits, k=4))
        resp = self.create_task(task_id=task_id)
        assert resp.json == {
            'code': 0,
            'msg': 'success',
            'data': {},
            'error': 0,
            'error_message': 'success'
        }
        # messages: List[Dict] = self.socketio_client.get_received("/")

        data = {"task_id": task_id, "ws_id": "0", "pallet_id": "1"}
        # 通知夹取位已到位
        resp = self.client.post(url_prefix + "/notice_place_ws_ready", json=data)
        assert resp.json == {
            'code': 0,
            'msg': 'success',
            'data': {},
            'error': 0,
            'error_message': 'success'
        }

        # 判断任务管理器里对应的抓取空间状态是否为True
        assert workspace_manager.get("0").is_ready

        # 判断HMI是否收到推送消息
        # messages: List[Dict] = self.socketio_client.get_received("/")
        # assert messages[0]["name"] == "order_info" and messages[0]["args"][0][
        #     "msg"] == "放置位0已到位"

        # 清空任务, 避免影响其他测试用例
        self.remove_task()

    def test_pallet_is_clear(self):
        """测试托盘已清空
        
        Returns:

        """
        # 异常输入
        resp = self.client.post(url_prefix + "/pallet_is_clear", json={})
        assert resp.json == {
            'code': 10400,
            'msg': 'Validation Error',
            'data': {},
            'error': 10400,
            'error_message': 'Input format is wrong: no pallet id'
        }, resp.json
        # message = self.socketio_client.get_received("/")[0]
        # assert (message["name"] == "order_info" and
        #         message["args"][0]["msg"] == resp.json[
        #             "error_message"]), message

        # 正常输入
        resp = self.client.post(
            url_prefix + "/pallet_is_clear",
            json={"pallet_id": "0"}
        )
        assert resp.json == {
            'code': 0,
            'msg': 'success',
            'data': {},
            'error': 0,
            'error_message': 'success'
        }, resp.json
        # message = self.socketio_client.get_received("/")[0]
        # assert (message["name"] == "order_info" and
        #         message["args"][0]["msg"] == "托盘0已清空")

    def test_single_class_depal_task(self):
        """测试单拆任务的发布
        
        Returns:

        """
        # 测试开始前, 先清空任务列表
        self.remove_task()

        api = url_prefix + "/single_class_depal_task"
        # 测试输入异常
        resp = self.client.post(api, json={})
        assert resp.json["code"] == 10400, resp.json
        # message = self.socketio_client.get_received("/")[0]
        # assert (message["name"] == "order_info" and
        #         message["args"][0]["msg"] == resp.json["error_message"])

        # 测试创建任务
        task_id = "123"
        resp = self.create_task(task_id)
        assert resp.json == {
            'code': 0,
            'msg': 'success',
            'data': {},
            'error': 0,
            'error_message': 'success'
        }
        # messages = self.socketio_client.get_received("/")
        # 会收到两次消息, 一条是更新批次的消息, 一条是订单事件消息
        # results = []
        # for message in messages:
        #     if message["name"] == "order_info":
        #         try:
        #             assert "单拆任务" in message["args"][0]["msg"], message
        #             results.append(True)
        #         except AssertionError:
        #             results.append(False)
        #     elif message["name"] == "batch_update":
        #         try:
        #             assert task_id == message["args"][0]["task_id"], message
        #             results.append(True)
        #         except AssertionError:
        #             results.append(False)
        # # 两条信息且两条都为真
        # assert len(results) == 1 and len(
        #     list(
        #         filter(
        #             lambda x: x,
        #             results
        #         )
        #     )
        # ) == 1, results

        # 检查TaskManager
        assert task_manager.get_task_by_id(task_id)
        task = task_manager.get_task_by_id(task_id)
        assert workspace_manager.get(task.from_ws).is_ready
        assert workspace_manager.get(task.to_ws).is_ready
        assert task.task_type == TaskType.SINGLE_DEPAL

        # 再次创建任务, 测试重复创建任务
        resp = self.create_task(task_id)
        assert resp.json["code"] == 12006, resp.json
        # message = self.socketio_client.get_received("/")[0]
        # assert (message["name"] == "order_info" and message["args"][0]["msg"]
        #         == resp.json["error_message"]), message

        # 清空任务
        self.remove_task()

    def test_more_size_of_task_manager(self):
        """测试创建超过task_manager容量的任务数."""
        self.remove_task()
        from wcs_adaptor.manager import task_manager

        task_manager.__TaskManager_size = 1
        resp = None
        for _ in range(11):
            resp = self.create_task(task_id=uuid.uuid4().__str__())
            if resp.json["code"] == 12013:
                break
        assert resp and resp.json["code"] == 12013, resp.json
        self.remove_task()

    def test_single_class_pal_task(self):
        """测试单码接口
        
        Returns:

        """
        # 测试开始前, 先清空任务列表
        self.remove_task()

        api = url_prefix + "/single_class_pal_task"
        # 测试输入异常
        resp = self.client.post(api, json={})
        assert resp.json["code"] == 10400, resp.json
        # message = self.socketio_client.get_received("/")[0]
        # assert (message["name"] == "order_info" and
        #         message["args"][0]["msg"] == resp.json["error_message"])

        # 测试创建任务
        task_id = "123"
        resp = self.create_task(task_id, path="/single_class_pal_task")
        assert resp.json == {
            'code': 0,
            'msg': 'success',
            'data': {},
            'error': 0,
            'error_message': 'success'
        }

        # messages = self.socketio_client.get_received("/")
        # 会收到两次消息, 一条是更新批次的消息, 一条是订单事件消息
        # results = []
        # for message in messages:
        #     if message["name"] == "order_info":
        #         try:
        #             assert "单码任务" in message["args"][0]["msg"], message
        #             results.append(True)
        #         except AssertionError:
        #             results.append(False)
        #     elif message["name"] == "batch_update":
        #         try:
        #             assert task_id == message["args"][0]["task_id"], message
        #             results.append(True)
        #         except AssertionError:
        #             results.append(False)
        # # 两条信息且两条都为真
        # assert len(results) == 1 and len(
        #     list(
        #         filter(
        #             lambda x: x,
        #             results
        #         )
        #     )
        # ) == 1, results

        # 检查TaskManager
        assert task_manager.get_task_by_id(task_id)
        task = task_manager.get_task_by_id(task_id)
        assert workspace_manager.get(task.from_ws).is_ready
        assert workspace_manager.get(task.to_ws).is_ready
        assert task.task_type == TaskType.SINGLE_PAL

        # 再次创建任务, 测试重复创建任务
        resp = self.create_task(task_id="123")
        assert resp.json["code"] == 12006, resp.json
        # message = self.socketio_client.get_received("/")[0]
        # assert (message["name"] == "order_info" and message["args"][0]["msg"]
        #         == resp.json["error_message"]), message

        # 清空任务
        self.remove_task()

    def test_multi_class_depal_task(self):
        """测试混拆接口
        
        Returns:

        """
        self.remove_task()

        api = url_prefix + "/multi_class_depal_task"
        # 测试输入异常
        resp = self.client.post(api, json={})
        assert resp.json["code"] == 10400, resp.json
        # message = self.socketio_client.get_received("/")[0]
        # assert (message["name"] == "order_info" and
        #         message["args"][0]["msg"] == resp.json["error_message"])

        # 测试创建任务
        task_id = "123"
        data = {
            "task_id": task_id,
            "sku_info": {},
            "from": "0",
            "to": "1",
        }
        resp = self.create_task(
            task_id,
            path="/multi_class_depal_task",
            data=data
        )
        assert resp.json == {
            'code': 0,
            'msg': 'success',
            'data': {},
            'error': 0,
            'error_message': 'success'
        }, resp.json

        # messages = self.socketio_client.get_received("/")
        # 会收到两次消息, 一条是更新批次的消息, 一条是订单事件消息
        # results = []
        # for message in messages:
        #     if message["name"] == "order_info":
        #         try:
        #             assert "混拆任务" in message["args"][0]["msg"], message
        #             results.append(True)
        #         except AssertionError:
        #             results.append(False)
        #     elif message["name"] == "batch_update":
        #         try:
        #             assert task_id == message["args"][0]["task_id"], message
        #             results.append(True)
        #         except AssertionError:
        #             results.append(False)
        # # 两条信息且两条都为真
        # assert len(results) == 1 and len(
        #     list(
        #         filter(
        #             lambda x: x,
        #             results
        #         )
        #     )
        # ) == 1, results

        assert task_manager.get_task_by_id(task_id)
        task = task_manager.get_task_by_id(task_id)
        assert workspace_manager.get(task.from_ws).is_ready is False
        assert workspace_manager.get(task.to_ws).is_ready is False
        assert task.task_type == TaskType.MULTI_DEPAL

        # 再次创建
        resp = self.create_task(task_id="123")
        assert resp.json["code"] == 12006, resp.json
        # message = self.socketio_client.get_received("/")[0]
        # assert (message["name"] == "order_info" and message["args"][0]["msg"]
        #         == resp.json["error_message"]), message

        # 清空任务
        self.remove_task()

    def test_multi_class_pal_task_online(self):
        """测试混码online接口."""
        self.remove_task()

        api = url_prefix + "/multi_class_pal_task_online"
        # 测试输入异常
        resp = self.client.post(api, json={})
        assert resp.json["code"] == 10400, resp.json
        # message = self.socketio_client.get_received("/")[0]
        # assert (message["name"] == "order_info" and
        #         message["args"][0]["msg"] == resp.json["error_message"])

        # 测试创建任务
        task_id = "123"
        resp = self.create_task(task_id, path="/multi_class_pal_task_online")
        assert resp.json == {
            'code': 0,
            'msg': 'success',
            'data': {},
            'error': 0,
            'error_message': 'success'
        }, resp.json

        # messages = self.socketio_client.get_received("/")
        # 会收到两次消息, 一条是更新批次的消息, 一条是订单事件消息
        # results = []
        # for message in messages:
        #     if message["name"] == "order_info":
        #         try:
        #             assert "混码任务" in message["args"][0]["msg"], message
        #             results.append(True)
        #         except AssertionError:
        #             results.append(False)
        # # 两条信息且两条都为真
        # assert len(results) == 1 and len(
        #     list(
        #         filter(
        #             lambda x: x,
        #             results
        #         )
        #     )
        # ) == 1, results

        assert task_manager.get_task_by_id(task_id)
        task = task_manager.get_task_by_id(task_id)
        assert workspace_manager.get(task.from_ws).is_ready is False
        assert workspace_manager.get(task.to_ws).is_ready is False
        assert task.task_type == TaskType.MULTI_PAL_ONLINE

        # 再次创建
        resp = self.create_task(task_id)
        assert resp.json["code"] == 12006, resp.json
        # message = self.socketio_client.get_received("/")[0]
        # assert (message["name"] == "order_info" and message["args"][0]["msg"]
        #         == resp.json["error_message"]), message

        # 清空任务
        self.remove_task()

    # def test_multi_class_pal_task_offline(self):
    #     """测试混码offline接口."""
    #     self.remove_task()
    #
    #     api = url_prefix + "/multi_class_pal_task/"
    #     # 测试输入异常
    #     resp = self.client.post(api, json={})
    #     assert resp.json["code"] == 10400, resp.json
    #     message = self.socketio_client.get_received("/")[0]
    #     assert (message["name"] == "order_info" and
    #             message["args"][0]["msg"] == resp.json["error_message"])
    #
    #     # 测试创建任务
    #     order_id = "123"
    #     data = {
    #         "task_id": order_id,
    #         "sku_info": [{
    #             "sku_id": "123",
    #             "length": 0.1,
    #             "width": 0.11,
    #             "height": 1.1,
    #             "weight": 1,
    #             "sku_num": 3,
    #             "sku_type": "1",
    #         }, {
    #             "sku_id": "456",
    #             "length": 1.1,
    #             "width": 0.11,
    #             "height": 0.2,
    #             "weight": 1,
    #             "sku_num": 3,
    #             "sku_type": "1",
    #         }, {
    #             "sku_id": "789",
    #             "length": 0.1,
    #             "width": 0.7,
    #             "height": 1.2,
    #             "weight": 3,
    #             "sku_num": 5,
    #             "sku_type": "1",
    #         }],
    #         "from": "0",
    #         "to": "1",
    #     }
    #     resp = self.create_task(
    #         order_id,
    #         path="/multi_class_pal_task/",
    #         data=data
    #     )
    #     # 计算规划结果
    #     plan_data, pallet_num = calculate_sku_order(data["sku_info"])
    #     assert resp.json == {
    #         'code': 0,
    #         'msg': 'success',
    #         'data': {
    #             'task_id': '123',
    #             'plan_data': [
    #                 {'sku_id': '456', 'serial_num': 0, 'pallet_index': 0},
    #                 {'sku_id': '456', 'serial_num': 1, 'pallet_index': 0},
    #                 {'sku_id': '456', 'serial_num': 2, 'pallet_index': 0},
    #                 {'sku_id': '789', 'serial_num': 3, 'pallet_index': 0},
    #                 {'sku_id': '789', 'serial_num': 4, 'pallet_index': 0},
    #                 {'sku_id': '789', 'serial_num': 5, 'pallet_index': 0},
    #                 {'sku_id': '789', 'serial_num': 6, 'pallet_index': 0},
    #                 {'sku_id': '789', 'serial_num': 7, 'pallet_index': 0},
    #                 {'sku_id': '123', 'serial_num': 8, 'pallet_index': 0},
    #                 {'sku_id': '123', 'serial_num': 9, 'pallet_index': 0},
    #                 {'sku_id': '123', 'serial_num': 10, 'pallet_index': 0}],
    #             'pallet_num': 2
    #         },
    #         'error': 0,
    #         'error_message': 'success'
    #     }, resp.json
    #     messages = self.socketio_client.get_received("/")
    #     # 会收到两次消息, 一条是更新批次的消息, 一条是订单事件消息
    #     results = []
    #     for message in messages:
    #         if message["name"] == "order_info":
    #             try:
    #                 assert "混码任务" in message["args"][0]["msg"], message
    #                 results.append(True)
    #             except AssertionError:
    #                 results.append(False)
    #     # 两条信息且两条都为真
    #     assert len(results) == 1 and len(
    #         list(
    #             filter(
    #                 lambda x: x,
    #                 results
    #             )
    #         )
    #     ) == 1, results
    #
    #     assert order_manager.get_order_by_id(order_id)
    #     order = order_manager.get_order_by_id(order_id)
    #     assert len(order.tasks) == len(plan_data)
    #     task = order.tasks[0]
    #     assert workspace_manager.get(task.from_ws).is_ready is False
    #     assert workspace_manager.get(task.to_ws).is_ready is False
    #     assert task.task_type == TaskType.MULTI_PAL_OFFLINE
    #
    #     # 再次创建
    #     resp = self.create_task(
    #         order_id,
    #         path="/multi_class_pal_task/",
    #         data=data
    #     )
    #     assert resp.json["code"] == 12007, resp.json
    #     message = self.socketio_client.get_received("/")[0]
    #     assert (message["name"] == "order_info" and message["args"][0]["msg"]
    #             == resp.json["error_message"]), message
    #
    #     # 清空任务
    #     self.remove_task()

    # def test_multi_class_pal_task_offline_start(self):
    #     """测试执行混码offline任务."""
    #     self.remove_task()
    #
    #     path = "/multi_class_pal_task/start"
    #     api = url_prefix + path
    #     # 测试输入异常
    #     resp = self.client.post(api, json={})
    #     assert resp.json["code"] == 10400, resp.json
    #     message = self.socketio_client.get_received("/")[0]
    #     assert (message["name"] == "order_info" and
    #             message["args"][0]["msg"] == resp.json["error_message"])
    #
    #     task_id = "123"
    #     # 在创建混码offline任务之前, 创建子任务
    #     # 抛出task_id的错误
    #     resp = self.create_task(
    #         task_id="123",
    #         path=path,
    #         data={
    #             "task_id": task_id,
    #             "sku_id": "123",
    #             "serial_num": 10
    #         }
    #     )
    #     assert resp.json == {
    #         'code': 12008,
    #         'msg': 'Order Not Found Error',
    #         'data': {},
    #         'error': 12008,
    #         'error_message': 'order_id: 123, 不存在.'
    #     }, resp.json
    #     message = self.socketio_client.get_received("/")[0]
    #     assert (message["name"] == "order_info" and
    #             message["args"][0]["msg"] == resp.json["error_message"])
    #
    #     # 1. 创建一个混码offline任务
    #     offline_data = {
    #         "task_id": task_id,
    #         "sku_info": [{
    #             "sku_id": "123",
    #             "length": 0.1,
    #             "width": 0.11,
    #             "height": 1.1,
    #             "weight": 1,
    #             "sku_num": 3,
    #             "sku_type": "1",
    #         }],
    #         "from": "0",
    #         "to": "1",
    #     }
    #     self.create_task(
    #         task_id,
    #         path="/multi_class_pal_task/",
    #         data=offline_data
    #     )
    #     self.socketio_client.get_received("/")
    #
    #     # 2.1 创建执行offline混码任务的任务, 传入错误的序列号
    #     data = {"task_id": task_id, "sku_id": "123", "serial_num": 10}
    #     resp = self.create_task(task_id, path=path, data=data)
    #     assert resp.json == {
    #         'code': 12002,
    #         "error": 12002,
    #         'error_message': 'task_id: 123_10, 未找到此任务.',
    #         'msg': 'Task Not Found Error',
    #         'data': {},
    #     }, resp.json
    #     # message = self.socketio_client.get_received("/")[0]
    #     # assert (message["name"] == "order_info" and message["args"][0]["msg"]
    #     #         == resp.json["error_message"]), message
    #
    #     # 传入错误的sku
    #     data = {"task_id": task_id, "sku_id": "xxxxxx", "serial_num": 0}
    #     resp = self.create_task(task_id, path=path, data=data)
    #     assert resp.json == {
    #         'code': 10400,
    #         'msg': 'Validation Error',
    #         'data': {},
    #         'error': 10400,
    #         'error_message': 'Serial num is wrong'
    #     }, resp.json
    #     # message = self.socketio_client.get_received("/")[0]
    #     # assert (message["name"] == "order_info" and message["args"][0]["msg"]
    #     #         == resp.json["error_message"]), message
    #
    #     # 2.2 创建执行offline混码任务的任务, 传入正确的序列号
    #     data = {"task_id": task_id, "sku_id": "123", "serial_num": 0}
    #     resp = self.create_task(task_id, path=path, data=data)
    #     assert resp.json == {
    #         'code': 0,
    #         'msg': 'success',
    #         'data': {},
    #         'error': 0,
    #         'error_message': 'success'
    #     }, resp.json
    #     # messages = self.socketio_client.get_received("/")
    #     # # 会收到两次消息, 一条是更新批次的消息, 一条是订单事件消息
    #     # results = []
    #     # for message in messages:
    #     #     if message["name"] == "order_info":
    #     #         try:
    #     #             assert "混码任务" in message["args"][0]["msg"], message
    #     #             results.append(True)
    #     #         except AssertionError:
    #     #             results.append(False)
    #     # # 两条信息且两条都为真
    #     # assert len(results) == 1 and len(
    #     #     list(
    #     #         filter(
    #     #             lambda x: x,
    #     #             results
    #     #         )
    #     #     )
    #     # ) == 1, results
    #
    #     assert order_manager.get_order_by_id(task_id)
    #     order = order_manager.get_order_by_id(task_id)
    #     task = order.tasks[0]
    #     assert workspace_manager.get(task.from_ws).is_ready is False
    #     assert workspace_manager.get(task.to_ws).is_ready is False
    #     assert task.task_type == TaskType.MULTI_PAL_OFFLINE
    #
    #     # 再次创建
    #     resp = self.create_task(
    #         task_id,
    #         path="/multi_class_pal_task/",
    #         data=offline_data
    #     )
    #     assert resp.json["code"] == 12007, resp.json
    #     # message = self.socketio_client.get_received("/")[0]
    #     # assert (message["name"] == "order_info" and message["args"][0]["msg"]
    #     #         == resp.json["error_message"]), message
    #
    #     # 清空任务
    #     self.remove_task()

    def create_task(
        self,
        task_id: str,
        path: str = "/single_class_depal_task",
        data: dict = None
    ) -> Response:
        """创建任务
        
        Args:
            task_id(str): 任务ID.
            path(str): 各个任务类型的路由. Default: "/single_class_depal_task".
            data(dict): 请求体. Default:
                {
                    "task_id": task_id,
                    "sku_info": {
                        "sku_id": "123",
                        "length": 1,
                        "width": 1,
                        "height": 1,
                        "weight": 1,
                        "sku_num": 1,
                        "sku_type": "1",
                    },
                    "from": "0",
                    "to": "1",
                }
        
        Returns:
            resp: 响应对象

        """
        if data is None:
            data = {
                "task_id": task_id,
                "sku_info": {
                    "sku_id": "123",
                    "length": 1,
                    "width": 1,
                    "height": 1,
                    "weight": 1,
                    "sku_num": 1,
                    "sku_type": "1",
                },
                "from": "0",
                "to": "1",
            }
        return self.client.post(url_prefix + path, json=data)

    # def test_backup_task(self):
    #     """测试任务备份"""
    #     from apps import db
    #
    #     db.init_app(app)
    #     model = db.session.query(TaskManagerModel).first()
    #     assert model, "任务为空"

    def remove_task(self):
        """清空任务."""
        resp = self.client.post("/api/cmd/remove_task")
        # self.socketio_client.get_received("/")
        assert resp.json == {
            'code': 0,
            'data': {},
            'error': 0,
            'error_message': '',
            'msg': '',
        }, resp.json
