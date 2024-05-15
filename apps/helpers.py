#  -*- coding: utf-8 -*-
#  Copyright (c) XYZ Robotics Inc. - All Rights Reserved
#  Unauthorized copying of this file, via any medium is strictly prohibited
#  Proprietary and confidential
#  Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022/8/10 下午2:17
import json
from typing import Any, Literal, Optional, Union, TYPE_CHECKING

import flask
from flask import Response, make_response
from pydantic import BaseModel

from apps.base_app.views.error_records.crud import crud_error_records
from apps.base_app.views.error_records.schemas import ErrorRecordsCreateSchema
from apps.enums import ErrorSources
from apps.models import start_transaction

if TYPE_CHECKING:
    from apps.ext.manager import TaskProto
    from apps.base_app.views.error_records.entity import ErrorRecordsEntity


def json_dumper(data) -> str:
    if isinstance(data, flask.Response):
        resp = data
        data = resp.json if resp.is_json else resp.data.decode("utf-8")
    elif isinstance(data, bytes):
        text = data.decode("utf-8")
        try:
            data = json.loads(text)
        except json.JSONDecodeError:
            data = text

    if isinstance(data, dict):
        data = json.dumps(
            data,
            ensure_ascii=False,
            indent=2
        )
    return data


def make_json_response(
    code: int = 0,
    msg: Optional[str] = None,
    data: Any = None,
    **extra_data
) -> Response:
    """构造json格式的http响应.

    Args:
        code: 状态码, 0正常, 非0异常.
        msg: 简短的消息.
        data: 默认None, 返回的详细数据, 接收可json序列化的任意类型.
        **extra_data: 额外数据.

    Returns:
        Response: flask的响应对象.

    """
    if msg is None:
        msg = "success" if code == 0 else "failed"

    # 向下兼容, error 和 error_message 字段保留.
    if "error" in extra_data and "error_message" not in extra_data:
        extra_data["error_message"] = msg

    js = json.dumps(
        {
            "code": code,
            "msg": msg,
            "data": {} if data is None else data,
            **extra_data
        }
    )
    response = make_response(js, 200)
    response.mimetype = "application/json"
    return response


def record_error(
    code: str,
    source: Union[Literal["xtf", "vision"], ErrorSources],
    task: Optional["TaskProto"] = None,
    msg: Optional[str] = None,
    tip: Optional[str] = None,
) -> "ErrorRecordsEntity":
    """记录一个异常到数据库.

    Args:
        code(str): 异常码.
        msg(str or None): 异常消息.
        source(str or ErrorSources): 异常来源，xtf 或 vision.
        task(BaseModel or None): 任务对象.
        tip(str or None): 异常提示.
    """
    create = ErrorRecordsCreateSchema(
        error_code=code,
        error_msg=msg,
        task=task and task.json(),
        error_source=ErrorSources(source) if isinstance(
            source,
            str
        ) else source,
        tip=tip,
    )
    with start_transaction() as session:
        return crud_error_records.create(
            session=session,
            create=create
        )
