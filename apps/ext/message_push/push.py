#  -*- coding: utf-8 -*-
#  Copyright (c) XYZ Robotics Inc. - All Rights Reserved
#  Unauthorized copying of this file, via any medium is strictly prohibited
#  Proprietary and confidential
#  Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022/9/14 下午2:43
import typing

from apps.enums import EventType
from apps.helpers import json_dumper
from .logger import logger
from .senders import (
    OrderInfoSender,
    MotionPlanLogSender,
    SystemLogSender,
    NodeErrorLogSender,
    RobotStatusSender,
    VisionSender,
    RGBLightSender,
    WCSConnectionStatusSender,
    NodeLogSender,
)

if typing.TYPE_CHECKING:
    from flask_socketio import SocketIO


class MessagePush:
    """消息推送.

    docs: https://161.189.84.82:8003/xyz-release-doc/ubuntu2004/hmi/latest/tutorial/message-sending/

    Examples:
        # 推送订单消息
        >>> from apps import mp
        # 推送常规消息
        >>> mp.order.info("xxx")
        # 推送异常消息(以红色高亮显示)
        >>> mp.order.error("xxx")
        # 推送 PLC 状态
        # 仅绿灯亮起
        >>> mp.rgb_light(green=1)
        # 红灯和黄灯亮起
        >>> mp.rgb_light(red=1, yellow=1)
    """

    def __init__(self, socketio: "SocketIO"):
        self.socketio = socketio
        self.order = OrderInfoSender()
        self.motion = MotionPlanLogSender()
        self.system = SystemLogSender()
        self.node_log = NodeLogSender()
        self.node_error_log = NodeErrorLogSender()
        self.robot_status = RobotStatusSender()
        self.vision = VisionSender()
        self.rgb_light = RGBLightSender()
        self.wcs_connection_status = WCSConnectionStatusSender()

    def push(self, event: str, message: dict):
        """发送消息.

        Args:
            event(EventType): 消息.
            message(Message): 消息对象.

        Raises:
            TypeError: event必须是EventType中的枚举值, 否则抛出异常.
            Exception: 未知异常

        """
        try:
            self.socketio.emit(event, message)
            text = json_dumper(message)
            logger.info(f"Event: {event}, Message: {text}")
        except Exception as exc:
            logger.error(f"error event: {event}", exc_info=True)
            raise exc
