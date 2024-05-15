# -*- coding: utf-8 -*-
"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022-04-24
"""
import time
import warnings
from functools import wraps

from pydantic import BaseModel, Field

from apps.enums import EventType
from apps.exceptions import XYZBaseError
from apps.helpers import json_dumper
from apps.log import customized_log as logger


class Message(BaseModel):
    time: int = Field(
        description="时间戳", default_factory=lambda: int(round(time.time() * 1000))
    )
    msg: str = Field(description="消息")
    status: bool = Field(description="是否正常")


# Deprecated
class HMIMessagePusher(object):
    """As HMI Bridge.

    Send message to HMI.

    Examples:
        Send Message:
        >>> from apps.globals import socketio
        >>> from apps.enums import EventType
        >>> mp = HMIMessagePusher(socketio)
        >>> mp.send(event=EventType.ORDER_INFO, data={"msg": "hello"}


    Attributes:
        socketio(flask_socketio.SocketIO): an instance of Socketio.

    """

    def __init__(self, socketio):
        self.socketio = socketio
        self.__deprecated_warning = False

    def _send(self, event: EventType, message: Message):
        """发送字典对象

        Args:
            event(EventType): 消息.
            message(Message): 消息对象.

        Examples:
            >>> from apps.services.message_push import Message
            >>> message = Message(msg="123", status=True)
            >>> hmi_msg_pusher._send(EventType.ORDER_INFO, message=message)

        Raises:
            TypeError: event必须是EventType中的枚举值, 否则抛出异常.
            Exception: 未知异常

        """
        if event not in EventType:
            raise TypeError("消息事件类型错误.")

        if not self.__deprecated_warning:
            warnings.warn(
                "`hmi_msg_pusher` 废弃警告，请使用 `mp` 代替，`mp` 使用文档：https://161.189.84.82:8003/xyz-release-doc/ubuntu2004/hmi/latest/tutorial/message-sending/",
                DeprecationWarning,
            )
            self.__deprecated_warning = True

        try:
            message = message.dict()
            self.socketio.emit(event.value, message)
            text = json_dumper(message)
            logger.info(f"[WebSocket] Event: {event.value}, Message: {text}")
        except Exception as exc:
            logger.error(f"[WebSocket] error event: {event.value}", exc_info=True)
            raise exc

    def push(self, event: EventType, msg: str, status: bool):
        """公共方法, 推送消息.

        Args:
            event(EventType): 事件类型.
            msg(str): 消息内容.
            status(bool): 状态, True or False.

        Examples:
            >>> hmi_msg_pusher.push(EventType.ORDER_INFO, "123456789", False)

        """
        self._send(event=event, message=Message(msg=msg, status=status))

    def monitor_error(self, func):
        """监控异常信息并发送至hmi.

        当前方法是一个装饰器.

        Examples:

            >>> import flask
            >>> from apps import hmi_msg_pusher
            >>> app = flask.Flask()
            >>> @app.route("/")
            ... @hmi_msg_pusher.monitor_error
            ... def demo():
            ...    raise XXXError(error_message="xxx")

        Returns:

        """

        @wraps(func)
        def decorator(*args, **kwargs):
            try:
                return func(*args, **kwargs)
            except XYZBaseError as err:
                data = Message(msg=err.error_message, status=False)
                self._send(event=err.msg_event, message=data)
                # 仅监控异常并发送信息, 并不处理异常, 异常继续抛出.
                raise err

        return decorator

    def push_order_info(self, message: str, status: bool = True):
        """发送订单信息

        Args:
            message(str): 消息
            status(bool): 是否正常, 默认为True.

        Returns:

        """
        self._send(
            event=EventType.ORDER_INFO, message=Message(msg=message, status=status)
        )

    def push_motion_plan_log(self, message: str, status: bool = True):
        """发送运动规划日志

        Args:
            message(str): 消息
            status(bool): 是否正常, 默认为True

        Returns:

        """
        self._send(
            event=EventType.MOTION_PLAN_LOG, message=Message(msg=message, status=status)
        )

    def push_system_log(self, message: str, status: bool = True):
        """发送系统日志

        Args:
            message(str): 消息
            status(bool): 状态, 默认为True

        Returns:

        """
        self._send(
            event=EventType.SYSTEM_LOG, message=Message(msg=message, status=status)
        )

    def push_error_message(self, error: XYZBaseError):
        """发送异常信息.

        Args:
            error(XYZBaseError): 异常对象.

        Examples:
            >>> from apps import create_app
            >>> from apps.enums import EventType
            >>> app = create_app()
            >>> ms = HMIMessagePusher(app.socketio)
            >>> error = XYZBaseError(error_message="xyz error.")
            >>> error.msg_event = EventType.ORDER_INFO
            >>> ms.push_error_message(error)
            >>> ms.push_error_message({})
            Traceback (most recent call last):
                ...
            AttributeError: 'dict' object has no attribute 'msg_event'

        Raises:
            TypeError: error必须是XYZBaseError的对象.

        """
        if not error.msg_event:
            raise TypeError("error required msg_event.")

        self._send(
            event=error.msg_event,
            message=Message(msg=error.error_message, status=False),
        )
