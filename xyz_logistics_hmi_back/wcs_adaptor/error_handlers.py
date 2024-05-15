# -*- coding: utf-8 -*-
"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022-04-24
"""
from flask import Response

from apps import create_app, make_json_response
from apps.exceptions import XYZBaseError
from apps.enums import EventType
from wcs_adaptor.exceptions import WCSError
from wcs_adaptor.depalletize.wcs import bp as wcs_bp
from wcs_adaptor.depalletize.rafcon import bp as rafcon_bp

app = create_app()


def handle_error(e):
    if e.msg_event:
        app.mp.order.error(e)
    return make_json_response(
        code=e.error_code,
        msg=e.name,
        data={},
        error=e.error_code,
        error_message=e.error_message,
    )


@app.errorhandler(WCSError)
def handle_wcs_error(e: WCSError) -> Response:
    """处理WCSError异常的通用方法.

    NOTICE: 此方法可以根据实际需求重写.

    Args:
        e(WCSError): a sub instance of WCSError.

    Returns:
        Response: flask的响应对象.

    """
    return handle_error(e)


def handle_xyz_error(e: XYZBaseError) -> Response:
    """XYZ后端异常的默认处理方法.

    NOTICE: 此方法可以根据实际需求重写.

    Args:
        e(XYZBaseError): a sub instance of XYZBaseError.

    Returns:
        Response: flask的响应对象.

    """
    e.msg_event = EventType.ORDER_INFO
    return handle_error(e)


wcs_bp.register_error_handler(XYZBaseError, handle_xyz_error)
rafcon_bp.register_error_handler(XYZBaseError, handle_xyz_error)
