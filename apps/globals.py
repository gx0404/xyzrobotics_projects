# -*- coding: utf-8 -*-
"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022-05-05

全局唯一的对象. 可在项目的任意位置通过`from apps.globals import xxx` 进行调用.

Examples:
    # 方式一:
    >>> from apps.globals import hmi_msg_pusher
    >>> from apps import create_app
    >>> app = create_app()
    >>> with app.app_context():
    ...     hmi_msg_pusher.push_order_info("123")

    # 方式二:
    >>> from apps import create_app
    >>> app = create_app()
    >>> app.hmi_msg_pusher.push_order_info("123")
"""
import typing
from functools import partial

from werkzeug.local import LocalProxy
from flask.globals import _app_ctx_err_msg, _find_app

from apps.openapi.openapi import OpenAPI

if typing.TYPE_CHECKING:
    from flask_socketio import SocketIO

    from apps.services.message_push import HMIMessagePusher
    from apps.ext.message_push import MessagePush
    from apps.utils.requesting.request_by_ws import Requesting
    from xyz_central_hub.client import HubClient


def lookup_app_object(name):
    app = _find_app()
    if app is None:
        raise RuntimeError(_app_ctx_err_msg)
    return getattr(app, name)


hmi_msg_pusher: "HMIMessagePusher" = LocalProxy(partial(lookup_app_object, "hmi_msg_pusher"))
socketio: "SocketIO" = LocalProxy(partial(lookup_app_object, "socketio"))
hub_client: "HubClient" = LocalProxy(partial(lookup_app_object, "hub_client"))
requesting: "Requesting" = LocalProxy(partial(lookup_app_object, "requesting"))
mp: "MessagePush" = LocalProxy(partial(lookup_app_object, "mp"))
openapi = OpenAPI()
