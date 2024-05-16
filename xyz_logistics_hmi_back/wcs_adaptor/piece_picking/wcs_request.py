#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    wcs_adaptor.piece_picking.wcs_request
    ~~~~~~~~~~~~~~~~~~~~~

    包含向上游WCS的接口请求

    Copyright (c) XYZ Robotics Inc. - All Rights Reserved
    Unauthorized copying of this file, via any medium is strictly prohibited
    Proprietary and confidential
    Developer: Chen Kun <kun.chen@xyzrobotics.ai>, May, 2022
"""
import uuid

import requests
from requests import RequestException, ConnectionError

from apps import settings
from apps.log import wcs_log
from wcs_adaptor.manager import order_manager, task_manager
from wcs_adaptor.piece_picking.utils import json_dumper

logger = wcs_log

# 系统状态反馈路径
SYSTEM_STATUS_PATH = "api/request/wcs/notice_system_is_ready"
# 单次分拣结果反馈路径
SINGLE_RESULT_PATH = "api/request/wcs/single_result"
# 单个任务结果反馈路径
TASK_RESULT_PATH = "api/request/wcs/task_result"
# 单个订单结果反馈路径
ORDER_RESULT_PATH = "api/request/wcs/order_result"

# 一些字符串的格式化
url_formatter = "http://{ip}:{port}/{path}"
request_formatter = "[{uid}][我方请求 {url} {method}]{message}"
response_formmater = "[{uid}][对方响应 {url} {method}]{message}"


def send_request(path, data, method='POST', timeout=(3, 5)):
    """封装requests的request请求方法，提供请求日志信息

    Args:
        path(str): 接口请求路径及相关参数，如GET请求的请求参数。
        data(dict): 接口请求数据，必须使用dict类型。
        method(str): (可选)默认使用POST请求。
        timeout(float or tuple(float, float)): (可选)设置请求的连接连接的默认时长为3秒，等待服务
            响应时间默认为5秒，防止请求一直阻塞导致程序卡死。

    Returns:
        response(None or requests.Response): 如果请求发生了异常，那么就返回None，如果请求成功，
            那么返回一个Response对象。
    """
    error_message = None
    uid = str(uuid.uuid1())[:8]
    url = url_formatter.format(
        ip=settings.WCS_IP, port=settings.WCS_PORT, path=path)
    logger.info(request_formatter.format(
        uid=uid,
        url=url,
        method=method,
        message=json_dumper(data)
    ))
    response = None
    try:
        response = requests.request(
            method=method, url=url, json=data, timeout=timeout)
    except ConnectionError:
        error_message = "连接失败"
    except RequestException as e:
        error_message = str(e)
    finally:
        if error_message is None:
            logger.info(response_formmater.format(
                uid=uid,
                url=url,
                method=method,
                message=json_dumper(response.content)
            ))
        else:
            logger.error(response_formmater.format(
                uid=uid,
                url=url,
                method=method,
                message=error_message
            ))
    return response


def notice_system_is_ready():
    """通知WCS本系统分拣服务已准备就绪。"""
    path = SYSTEM_STATUS_PATH
    data = {
        "status": 0,
        "status_name": "ready"
    }
    response = send_request(path, data)
    return response


def feedback_single_result(task=None):
    """向WCS反馈单次分拣结果。"""
    path = SINGLE_RESULT_PATH
    if task is None:
        task = task_manager.first()
    if task is None:
        return
    data = {"task_id": task.task_id}
    response = send_request(path, data)
    return response


def feedback_task_result(task=None):
    """向WCS反馈任务结果。"""
    path = TASK_RESULT_PATH
    if task is None:
        task = task_manager.first()
    if task is None:
        return
    data = {"task_id": task.task_id}
    response = send_request(path, data)
    return response


def feedback_order_result(order=None):
    """向WCS反馈订单结果。"""
    path = ORDER_RESULT_PATH
    if order is None:
        order = order_manager.first()
    if order is None:
        return
    data = {"order_id": order.order_id}
    response = send_request(path, data)
    return response
