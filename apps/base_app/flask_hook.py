#!usr/bin/env python
# -*- coding: utf-8 -*-
'''
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Xiao Ao <ao.xiao@xyzrobotics.ai>, January, 2021
'''
from flask import request
from functools import wraps

from apps.log import outside_log
from apps.settings import flask_space
from apps.helpers import make_json_response, json_dumper


# TODO(Yuhang Wu): 命名风格调整, ignoreReq -> ignore_req
def req_log(ignoreReq=False, ignoreRes=False, log=outside_log):
    """Log all key messages of the request and resposne.

    Args:
        ignoreReq: bool, represents ignoring request.data in log, better to set it false in GET method.
        ignoreRes: bool, represents ignoring return data.
        log: logger object
    """

    def _req_log(func):
        @wraps(func)
        def _wrapper(*args, **kwargs):
            msg = f"{request.remote_addr} - {request.path}"
            if ignoreReq:
                log.info(f"request  - {msg} - [...]")
            else:
                text = []
                if request.query_string:
                    b = json_dumper(request.query_string)
                    text.append(f"Query: {b}")
                if request.data:
                    body = request.data
                    b = json_dumper(body)
                    text.append(f"Body: {b}")
                if request.files:
                    text.append(f"Files: {request.files}")
                if request.form:
                    text.append(f"Form: {request.form}")
                log.info(f"request  - {msg} - {','.join(text)}")

            try:
                response = func(*args, **kwargs)
            except Exception as e:
                log.warning(f"response - {msg} - {repr(e)}", exc_info=True)
                raise e

            if ignoreRes:
                log.info(f"response - {msg} - [...]")
            else:
                log.info(f"response - {msg} - {json_dumper(response)}")
            return response

        return _wrapper

    return _req_log


def big_event(event, log=outside_log):
    def _wrapper(func):
        @wraps(func)
        def __wrapper(*args, **kwargs):
            pre_event = "PRE_" + event
            for event_func in flask_space.get(pre_event, []):
                log.info(
                    u"[CMD EVENT] In %s, call %s" % (pre_event, str(event_func))
                )
                event_func()
            ret = func(*args, **kwargs)
            post_event = "POST_" + event
            for event_func in flask_space.get(post_event, []):
                log.info(
                    u"[CMD EVENT] In %s, call %s" % (
                        post_event, str(event_func))
                )
                event_func()
            return ret

        return __wrapper

    return _wrapper
