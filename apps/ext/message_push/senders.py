#  -*- coding: utf-8 -*-
#  Copyright (c) XYZ Robotics Inc. - All Rights Reserved
#  Unauthorized copying of this file, via any medium is strictly prohibited
#  Proprietary and confidential
#  Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022/9/14 下午2:47
import time
import typing

from .logger import logger
from apps.helpers import json_dumper
from apps.enums import RobotStatus, EventType
from apps.exceptions import XYZBaseError
from apps.models import Log, db


class BaseSender:
    """消息处理基类."""

    event_type: typing.Optional[EventType] = None
    show_log: bool = False

    @staticmethod
    def handle(message: dict) -> str:
        """处理消息."""
        return json_dumper(message)

    def send(self, message: dict) -> None:
        """发送消息."""
        from apps.app import cached_app

        app = cached_app()
        try:
            app.socketio.emit(self.get_message_type(), message)
            text = self.handle(message)
            self.show_log and logger.info(f"Event: {self.get_message_type()}, Message: {text}")
        except Exception as exc:
            self.show_log and logger.error(f"error event: {self.get_message_type()}", exc_info=True)
            raise exc

    def get_event_type(self):
        """获取事件类型."""
        return self.event_type

    def get_message_type(self):
        """获取消息类型."""
        return self.event_type.value

    def __call__(self, *args, **kwargs):
        raise NotImplemented


class OrderInfoSender(BaseSender):
    """订单信息消息处理."""

    def __init__(self):
        self.event_type = EventType.ORDER_INFO
        self.show_log = True

    def info(self, message: str):
        """发送正常状态的订单消息."""
        self.send(
            {"time": int(round(time.time() * 1000)), "msg": message, "status": True}
        )

    def error(self, message: typing.Union[XYZBaseError, str]):
        """发送异常订单消息."""
        if isinstance(message, XYZBaseError):
            message = message.error_message
        self.send(
            {"time": int(round(time.time() * 1000)), "msg": message, "status": False}
        )

    def __call__(self, message: str, status: bool = True):
        """发送正常订单消息."""
        self.send(
            {"time": int(round(time.time() * 1000)), "msg": message, "status": status}
        )


class MotionPlanLogSender(BaseSender):
    """运动规划日志消息处理."""

    def __init__(self):
        self.event_type = EventType.MOTION_PLAN_LOG

    def __call__(self, message: dict):
        self.send(message)


class SystemLogSender(BaseSender):
    """系统日志消息处理."""

    def __init__(self):
        self.event_type = EventType.SYSTEM_LOG
        self.show_log = True

    def send(self, message: dict) -> None:
        super(SystemLogSender, self).send(message)
        self.save_log(message)

    @staticmethod
    def save_log(data: dict):
        """保存日志到数据库."""
        from apps import cached_app

        log = Log(
            msg_type=data["msg_type"],
            code=data["code"],
            class_=data.pop("class", "sys"),
            tag=data.pop("tag", "sys"),
            object_=data.pop("object", ""),
            tool=data.pop("tool", ""),
            worksapce=data.pop("workspace", ""),
            zh_msg=data.get("zh_msg", ""),
            en_msg=data.get("en_msg", ""),
            ja_msg=data.get("ja_msg", ""),
            zh_tip=data.get("zh_tip", ""),
            en_tip=data.get("en_tip", ""),
            ja_tip=data.get("ja_tip", ""),
            timeout=data.get("timeout", None),
            timestamp=data.pop("timestamp", time.time()),
        )

        with cached_app().app_context():
            db.session.add(log)
            db.session.commit()

    def error(self, message: dict, error_handle_method: int = 0):
        """错误日志."""
        if "error_handle_method" not in message:
            message["error_handle_method"] = error_handle_method
        message["msg_type"] = "error"
        self.send(message)

    def warning(self, message: dict, error_handle_method: int = 0):
        """警告日志."""
        if "error_handle_method" not in message:
            message["error_handle_method"] = error_handle_method
        message["msg_type"] = "warning"
        self.send(message)

    def warn(self, message: dict, error_handle_method: int = 0):
        """警告日志."""
        self.warning(message, error_handle_method)

    def info(self, message: dict, error_handle_method: int = 0):
        """信息日志."""
        if "error_handle_method" not in message:
            message["error_handle_method"] = error_handle_method
        message["msg_type"] = "info"
        self.send(message)

    def __call__(self, message: dict):
        self.send(message)


class NodeErrorLogSender(BaseSender):
    """节点异常日志发送"""

    def __init__(self):
        self.event_type = EventType.NODE_ERROR_EVENT

    def __call__(self, message: dict):
        self.send(message=message)


class RobotStatusSender(BaseSender):
    """机器人状态发送"""

    def __init__(self):
        self.event_type = EventType.ROBOT_STATUS
        self.show_log = True

    def set_import_io_error(self):
        """设置导入异常状态"""
        self(RobotStatus.IMPORT_IO_ERROR)

    def set_plc_connection_error(self):
        """设置 PLC 连接异常"""
        self(RobotStatus.PLC_CONNECTION_ERROR)

    def set_stopped(self):
        """设置已停止状态"""
        self(RobotStatus.STOPPED)

    def set_normal(self):
        """设置正常状态"""
        self(RobotStatus.NORMAL)

    def set_paused(self):
        """设置已暂停状态"""
        self(RobotStatus.PAUSED)

    def set_estop(self):
        """设置急停状态"""
        self(RobotStatus.ESTOP)

    def set_error(self):
        """设置异常状态"""
        self(RobotStatus.ERROR)

    def set_collision(self):
        """设置碰撞状态"""
        self(RobotStatus.COLLISION)

    def set_manual(self):
        """设置手动状态"""
        self(RobotStatus.MANUAL)

    def set_disabled(self):
        """设置禁用状态"""
        self(RobotStatus.DISABLED)

    def set_not_cycle(self):
        """设置 not_cycle 状态"""
        self(RobotStatus.NOT_CYCLE)

    def set_unknown(self):
        """设置未知状态"""
        self(RobotStatus.UNKNOWN)

    def __call__(self, status: typing.Union[RobotStatus, str]):
        self.send({"status": status if isinstance(status, str) else status.value})


class VisionSender(BaseSender):
    """视觉节点消息发送器"""

    def __init__(self):
        self.event_type = EventType.VISION

    def active(self, data):
        """发送视觉节点激活消息"""
        self.send({"active_vision": data})

    def passive(self, data):
        self.send({"passive_vision": data})

    def place(self, data):
        self.send({"place_vision": data})

    def __call__(self, data):
        self.send(data)


class RGBLightSender(BaseSender):
    """RGB三色灯发送器"""

    def __init__(self):
        self.event_type = EventType.RGB_LIGHT
        self.show_log = True

    def __call__(
        self,
        green: typing.Union[int, bool] = 0,
        yellow: typing.Union[int, bool] = 0,
        red: typing.Union[int, bool] = 0,
    ):
        self.send({"data": [int(bool(green)), int(bool(yellow)), int(bool(red))]})


class WCSConnectionStatusSender(BaseSender):
    """WCS 连接状态发送器"""

    def __init__(self):
        self.event_type = EventType.WCS_CONNECTION

    def connected(self):
        """连接状态"""
        self(True)

    def disconnected(self):
        """断开状态"""
        self(False)

    def __call__(self, status: bool):
        self.send({"connection": status})


class NodeLogSender(BaseSender):
    """节点日志发送器"""

    def __init__(self):
        self.event_type = EventType.NODE_LOG

    def __call__(self, node_name: str, data):
        self.send({"node_name": node_name, "message": data})
