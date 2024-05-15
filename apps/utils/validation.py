# -*- coding: utf-8 -*-
"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022-05-27
"""
import json
from functools import wraps
from typing import Callable, Optional, Type, Union

import pydantic
from flask import Response, request
from flask_pydantic.converters import convert_query_params
from flask_pydantic.core import (
    get_body_dict,
    is_iterable_of_models,
    make_json_response, unsupported_media_type_response,
    validate_many_models,
    validate_path_params
)
from flask_pydantic.exceptions import (
    InvalidIterableOfModelsException, JsonBodyParsingError,
    ManyModelValidationError
)
from marshmallow.schema import Schema
from pydantic import BaseModel, ValidationError

from apps.base_app.views.hmi.schemas import CustomizedHmiSchema
from apps.exceptions import (
    XYZCustomAPIMissingError,
    XYZValidationError,
    XYZUnsupportedMediaType
)
from apps.settings import settings
from apps.utils.pydantic_from_marshmallow import pydantic_from_marshmallow

cached_model_from_schema = {}


def validator(
    body: Union[None, Type[BaseModel], Type[Schema]] = None,
    query: Union[None, Type[BaseModel], Type[Schema]] = None,
    on_success_status: int = 200,
    exclude_none: bool = False,
    response_many: bool = False,
    request_body_many: bool = False,
    response_by_alias: bool = False,
    get_json_params: Optional[dict] = None,
    push_msg_if_error: bool = True,
) -> Callable:
    """

    Examples:
        >>> import json
        >>> from apps import create_app
        >>> app = create_app(testing=True)
        >>> client = app.test_client()
        >>> class InputBody(BaseModel):
        ...     task_id: str
        >>> @app.route("/test_model", methods=["GET", "POST"])
        ... @validator()
        ... def test_model(body: InputBody):
        ...     return {'task_id': {'value': body.task_id, 'type': str(type(body.task_id))}}
        >>> client = app.test_client()
        >>> text = client.post("/test_model", json={"task_id": "123"}).data.decode()
        >>> json.loads(text)
        {'task_id': {'type': "<class 'str'>", 'value': '123'}}

        >>> from marshmallow import Schema, fields
        >>> class InputSchema(Schema):
        ...     task_id = fields.String(required=True)
        >>> @app.route("/test_schema", methods=["GET", "POST"])
        ... @validator()
        ... def test_schema(body: InputSchema):
        ...     return {'task_id': {'value': body.task_id, 'type': str(type(body.task_id))}}
        >>> client = app.test_client()
        >>> text = client.post("/test_schema", json={"task_id": "123"}).data.decode()
        >>> json.loads(text)
        {'task_id': {'type': "<class 'str'>", 'value': '123'}}

    Args:
        body(Optional[Type[BaseModel]] or Optional[Type[Schema]]): Default: None, 请求体数据模型类.
        query(Optional[BaseModel] or Optional[Schema]): Default: None, url查询数据模型.
        on_success_status(int): Default: 200, 当成功响应时的状态码.
        exclude_none(bool): Default: False, 是否排除值为None的字段。
        response_many(bool): Default: False, 是否将响应以列表形式返回.
        request_body_many(bool): Default: False, 请求体中的内容是否是多个object组成的列表.
        response_by_alias(bool): Default: False, 被装饰函数返回值是BaseModel类型时有效, 是否以别名返回数据.
        get_json_params(Optional[dict]): Default: None, 指定获取输入中的某些字段用以校验.
        push_msg_if_error(bool): Default: True, 当发生校验异常时, 是否发送消息.

    Returns:
        Response: 返回flask的响应对象.

    Raises:
        XYZValidationError: 校验失败时抛出异常.

    """

    def decorate(func: Callable) -> Callable:
        @wraps(func)
        def wrapper(*args, **kwargs):
            q, b, err = None, None, {}
            kwargs, path_err = validate_path_params(func, kwargs)
            if path_err:
                err["path_params"] = path_err
            query_in_kwargs = func.__annotations__.get("query")
            query_model = query_in_kwargs or query
            if query_model:
                if issubclass(query_model, BaseModel):
                    query_params = convert_query_params(
                        request.args,
                        query_model
                    )
                elif issubclass(query_model, Schema):
                    query_params = request.args.to_dict()
                    query_model = pydantic_from_marshmallow(query_model)
                else:
                    raise XYZValidationError(
                        "Model must be a BaseModel or Schema."
                    )

                try:
                    q = query_model(**query_params)
                except ValidationError as ve:
                    err["query_params"] = ve.errors()

            body_in_kwargs = func.__annotations__.get("body")
            body_model = body_in_kwargs or body
            body_params = get_body_dict(**(get_json_params or {}))

            if body_model:
                if issubclass(body_model, BaseModel):
                    if "__root__" in body_model.__fields__:
                        try:
                            b = body_model(__root__=body_params).__root__
                        except ValidationError as ve:
                            err["body_params"] = ve.errors()
                    elif request_body_many:
                        try:
                            b = validate_many_models(body_model, body_params)
                        except ManyModelValidationError as e:
                            err["body_params"] = e.errors()
                    else:
                        try:
                            b = body_model(**body_params)
                        except TypeError as e:
                            content_type = request.headers.get(
                                "Content-Type",
                                ""
                            ).lower()
                            media_type = content_type.split(";")[0]
                            if media_type != "application/json":
                                raise XYZUnsupportedMediaType(request_cont_type=media_type) from e
                            else:
                                raise JsonBodyParsingError() from e
                        except ValidationError as ve:
                            err["body_params"] = json.loads(ve.json())
                elif issubclass(body_model, Schema):
                    _schema = body_model
                    if _schema in cached_model_from_schema:
                        body_model = cached_model_from_schema[_schema]
                    else:
                        body_model = pydantic_from_marshmallow(_schema)
                        # 缓存schema与model，减少转化次数.
                        cached_model_from_schema[_schema] = body_model

                    try:
                        b = body_model.parse_obj(body_params)
                    except ValidationError as ve:
                        err["body_params"] = json.loads(ve.json())
                else:
                    raise XYZValidationError(
                        "Model must be a BaseModel or Schema."
                    )

            request.query_params = q
            request.body_params = b
            if query_in_kwargs:
                kwargs["query"] = q
            if body_in_kwargs:
                kwargs["body"] = b

            if err:
                error = XYZValidationError(
                    error_message=str(err.get("body_params", None) or err)
                )
                if not push_msg_if_error:
                    error.msg_event = None
                raise error

            res = func(*args, **kwargs)

            if response_many:
                if is_iterable_of_models(res):
                    return make_json_response(
                        res,
                        on_success_status,
                        by_alias=response_by_alias,
                        exclude_none=exclude_none,
                        many=True,
                    )
                else:
                    raise InvalidIterableOfModelsException(res)

            if isinstance(res, BaseModel):
                return make_json_response(
                    res,
                    on_success_status,
                    exclude_none=exclude_none,
                    by_alias=response_by_alias,
                )

            if (
                isinstance(res, tuple)
                and len(res) == 2
                and isinstance(res[0], BaseModel)
            ):
                return make_json_response(
                    res[0],
                    res[1],
                    exclude_none=exclude_none,
                    by_alias=response_by_alias,
                )

            return res

        return wrapper

    return decorate


def validate(
    model: Union[Type[BaseModel], Type[Schema]],
    data: dict,
    push_msg_if_error: bool = True
) -> dict:
    """校验数据格式是否符合数据类.

    Examples:
        >>> from marshmallow import Schema, fields
        >>> class TestSchema(Schema):
        ...     a = fields.String(required=True)
        >>> validate(TestSchema, {"a": "1"})
        {'a': '1'}
        >>> validate(TestSchema, {"b": 1})
        Traceback (most recent call last):
            ...
        apps.exceptions.XYZValidationError: < error_code: 10400, name: Validation Error, message: {'a': ['Missing data for required field.']} >
        >>> from pydantic import BaseModel
        >>> class TestModel(BaseModel):
        ...     a: str
        >>> validate(TestModel, {"a": "1"})
        {'a': '1'}
        >>> validate(TestModel, {})
        Traceback (most recent call last):
            ...
        apps.exceptions.XYZValidationError: < error_code: 10400, name: Validation Error, message: 1 validation error for TestModel
        a
          field required (type=value_error.missing) >

    Args:
        model: 校验类, marshmallow.Schema or pydantic.BaseModel.
        data: 待校验的数据.
        push_msg_if_error: 是否推送异常信息至HMI.

    Returns:
        dict: 用于校验的数据

    Raises:
        XYZValidationError: 校验失败时抛出.

    """
    error = None
    if issubclass(model, BaseModel):
        try:
            model.parse_obj(data)
        except pydantic.ValidationError as err:
            error = XYZValidationError(error_message=str(err))
    elif issubclass(model, Schema):
        data, errors = model().load(data)
        if errors:
            error = XYZValidationError(error_message=str(errors))
    else:
        raise TypeError("model must be a Schema or a BaseModel.")

    if error:
        if not push_msg_if_error:
            error.msg_event = None
        raise error
    return data


def validate_custom_hmi_api_existed(app):
    """ Validate required apis existed

    Args: app(application)

    """
    # Validate hmi params first
    with open(settings.CUSTOM_HMI_CG, "r") as fin:
        custom_hmi_params = json.load(fin)
    validate(CustomizedHmiSchema, custom_hmi_params)

    # Validate route existed
    prefix = custom_hmi_params["url_prefix"]
    all_route = {f"{rule}" for rule in app.url_map.iter_rules()}
    for hmi_content in custom_hmi_params["content"]:
        custom_route = prefix + hmi_content["route"]
        if custom_route not in all_route:
            raise XYZCustomAPIMissingError(route=custom_route)


if __name__ == '__main__':
    from marshmallow import Schema, fields


    class TestSchema(Schema):
        a = fields.String()


    validate(TestSchema, {"a": "1"})
    validate(TestSchema, {"b": 1})
