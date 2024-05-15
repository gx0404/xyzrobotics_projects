#  -*- coding: utf-8 -*-
#  Copyright (c) XYZ Robotics Inc. - All Rights Reserved
#  Unauthorized copying of this file, via any medium is strictly prohibited
#  Proprietary and confidential
#  Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022/10/11 下午5:27
import unittest
import uuid

from apps import create_app


class TestMessagePush(unittest.TestCase):
    """测试消息推送"""

    # def __init__(self, *args, **kwargs):
    #     super(TestMessagePush, self).__init__(*args, **kwargs)

    def setUp(self) -> None:
        self.app = create_app(testing=True)
        self.client = self.app.test_client()
        self.socketio_client = self.app.socketio.test_client(
            self.app, flask_test_client=self.client
        )
        self.mp = self.app.mp

    def test_order_info(self):
        """测试订单消息推送"""
        # 推送正常消息
        self.mp.order.info("message")
        messages = self.socketio_client.get_received("/")
        assert messages[0]["name"] == "order_info"
        data = messages[0]["args"][0]
        assert data["msg"] == "message" and data["status"] == True

        # 推送异常消息
        self.mp.order.error("error message")
        messages = self.socketio_client.get_received("/")
        data = messages[0]["args"][0]
        assert data["msg"] == "error message" and data["status"] == False

    def test_robot_status(self):
        """测试机器人状态推送"""
        # 修改为正常状态
        self.mp.robot_status.set_normal()
        messages = self.socketio_client.get_received("/")

        assert messages[0]["name"] == "RobotStatus"
        data = messages[0]["args"][0]
        assert data["status"] == "NORMAL", data

        # 修改为异常状态
        self.mp.robot_status.set_error()
        messages = self.socketio_client.get_received("/")
        data = messages[0]["args"][0]
        assert data["status"] == "ERROR", data

    def test_node_error_log(self):
        """测试节点异常日志推送"""
        self.mp.node_error_log({"key": "value"})
        messages = self.socketio_client.get_received("/")
        assert messages[0]["name"] == "node_error", messages
        data = messages[0]["args"][0]
        assert data == {"key": "value"}, data

    def test_system_log(self):
        """测试系统日志."""
        uid = uuid.uuid4().__str__().rsplit("-")[-1][:4]

        # 推送异常弹窗
        error_data = {
            "code": uid,
            "msg_type": "error",
            "en_msg": "en",
            "zh_msg": "zh",
            "zh_tip": "zh_tip",
            "ja_msg": "ja_msg",
            "ja_tip": "ja_tip",
            "class": "motion",
        }
        self.mp.system.error(error_data)
        messages = self.socketio_client.get_received("/")

        assert messages[0]["name"] == "system_log", messages
        data = messages[0]["args"][0]
        assert data == {
            "code": uid,
            "msg_type": "error",
            "en_msg": "en",
            "zh_msg": "zh",
            "zh_tip": "zh_tip",
            "ja_msg": "ja_msg",
            "ja_tip": "ja_tip",
            "error_handle_method": 0,
        }, data

        # 查看数据库是否有上面这条记录
        from apps import db
        from apps.models.log import Log

        with self.app.app_context():
            queryset = db.session.query(Log).filter(Log.code == uid).first()
            assert queryset.code == uid, queryset.code

    def test_motion_log(self):
        """测试 Motion Plan Log 日志推送"""
        self.mp.motion({"key": "value"})
        messages = self.socketio_client.get_received("/")
        assert messages[0]["name"] == "motion_plan_log"
        data = messages[0]["args"][0]
        assert data == {"key": "value"}
