#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    wcs_adaptor.piece_picking.utils
    ~~~~~~~~~~~~~~~~~~~~~

    提供分拣站后端中常用的功能函数

    Copyright (c) XYZ Robotics Inc. - All Rights Reserved
    Unauthorized copying of this file, via any medium is strictly prohibited
    Proprietary and confidential
    Developer: Chen Kun <kun.chen@xyzrobotics.ai>, May, 2022
"""
import uuid
import functools
import json

import flask
from flask import request

from apps.log import outside_log

request_formatter = "[{uid}][对方请求 {url} {method} {remote_addr} {content_type}]{message}"
response_formatter = "[{uid}][我方响应 {url} {method} {remote_addr} {content_type}]{message}"


def standard_response(exception=None):
    if exception is not None:
        return {
            "code": -1,
            "msg": 'error',
            "data": str(exception)
        }


def catch_request_log(
    ignoreReq=False,
    ignoreRes=False,
    response_func=standard_response,
    logger=outside_log,
    indent=2
):
    """捕获请求时产生的异常，及正常请求日志，如果发生异常，则按传入的response_func格式返回错误信息

    Args:
        ignoreReq(bool): (可选)represents ignoring request.data in log, better to set it false in GET method.
        ignoreRes(bool): (可选)represents ignoring return data.
        response_func(func): (可选)当发生装饰的函数发生异常时，抓取异常，并返回标准格式的响应的异常信息
        logger(Handler): (可选)对日志处理类进行参数化
        indent(int or None): (可选)设置响应数据时的日志显示格式，是否带有格式化缩进，如果为None，则不添加缩进

    """
    def _req_log(func):
        @functools.wraps(func)
        def _wrapper(*args, **kwargs):
            uid = str(uuid.uuid1())[:8]
            if ignoreReq:
                message = ""
            elif request.data:
                message = json_dumper(request.data, indent=indent)
            elif request.form:
                message = json_dumper(request.form, indent=indent)
            else:
                message = ""
            logger.info(request_formatter.format(
                uid=uid,
                url=request.url,
                method=request.method,
                remote_addr=request.remote_addr,
                content_type=request.content_type,
                message=message
            ))

            try:
                ret = func(*args, **kwargs)
            except Exception as e:
                ret = response_func(exception=e)
                logger.error(response_formatter.format(
                    uid=uid,
                    url=request.url,
                    method=request.method,
                    remote_addr=request.remote_addr,
                    content_type=request.content_type,
                    message=json_dumper(ret, indent=indent)
                ), exc_info=True)
            else:
                if not ret or ignoreRes:
                    message = ""
                else:
                    message = json_dumper(ret, indent=indent)
                logger.info(response_formatter.format(
                    uid=uid,
                    url=request.url,
                    method=request.method,
                    remote_addr=request.remote_addr,
                    content_type=request.content_type,
                    message=message
                ))
            return ret
        return _wrapper
    return _req_log


def json_dumper(data, indent=2):
    """将python的dict类型json序列化为字符串，不再考虑Py2的兼容问题，因为部分
    代码中已经使用了参数类型声明，在Py2中会报错

    Args:
        data(dict): 需要格式化的数据
        indent(int): (可选)默认缩进2个空格

    Returns:
        result(str): 返回序列化后的字符串
    """
    result = ""
    if isinstance(data, flask.Response):
        result = data.data.decode("utf-8")
    else:
        if isinstance(data, bytes):
            result = data.decode("utf-8")
        elif isinstance(data, dict):
            result = json.dumps(data, ensure_ascii=False, indent=indent)
    return result
