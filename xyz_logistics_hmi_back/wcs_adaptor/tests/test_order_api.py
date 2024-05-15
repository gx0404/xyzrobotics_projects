# -*- coding: utf-8 -*-
"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022-07-01

用于测试Order相关功能的测试用例.
"""
import unittest

from flask import url_for

from apps import create_app
from wcs_adaptor import init_app
from wcs_adaptor.entity import LiteOrder as Order
from wcs_adaptor.manager import order_manager


class TestOrder(unittest.TestCase):

    def setUp(self) -> None:
        self.app = create_app(testing=True)
        init_app(self.app)
        self.client = self.app.test_client()

    def remove_all_orders(self):
        """清空订单管理器中的订单."""
        order_manager.clear()

    def create_order(self, order_id):
        """创建一个订单"""
        order = Order(order_id=order_id)
        order_manager.add(order=order)

    def test_get_orders_from_manager(self):
        """测试从订单管理器中获取多个订单."""
        self.remove_all_orders()
        with self.app.app_context(), self.app.test_request_context():
            api = url_for("order.list_orders")
            # 获取一个空订单列表
            rv = self.client.get(api)
            assert rv.json["data"] == [], rv.json

            # 创建一个订单
            self.create_order(order_id="123")

            # 获取一个有数据的订单列表
            rv = self.client.get(api)
            assert rv.json["data"][0]['order_id'] == '123'
        self.remove_all_orders()

    def test_get_one_order_from_manager(self):
        """测试从订单管理器中获取指定订单ID的订单."""
        self.remove_all_orders()
        with self.app.app_context(), self.app.test_request_context():
            api = url_for("order.list_orders")
            # 获取空数据
            rv = self.client.get(api)
            assert rv.json["data"] == [], rv.json

            # 创建订单
            self.create_order(order_id="123")

            # 获取一个有数据的订单
            rv = self.client.get(url_for("order.get_order", order_id="123"))
            assert rv.json["data"]["order_id"] == "123"
        self.remove_all_orders()
