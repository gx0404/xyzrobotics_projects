#  -*- coding: utf-8 -*-
#  -*- coding: utf-8 -*-
#  Copyright (c) XYZ Robotics Inc. - All Rights Reserved
#  Unauthorized copying of this file, via any medium is strictly prohibited
#  Proprietary and confidential
#  Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022/11/15 下午1:40
from typing import Type, Union

from marshmallow import Schema
from pydantic import BaseModel, Field, root_validator

from .utils import create_model, pydantic_from_marshmallow


class Response(BaseModel):
    """Response model for OpenAPI."""

    description: str
    content: dict
    headers: dict = None
    links: dict = None

    class Config:
        schema_extra = {
            "example": {
                "description": "Successful response",
                "content": {
                    "application/json": {
                        "schema": {
                            "type": "object",
                            "properties": {
                                "code": {"type": "integer", "example": 200},
                                "msg": {"type": "string", "example": "success"},
                                "data": {"type": "object", "example": {}},
                            },
                        }
                    }
                },
            }
        }


class DefaultResponse(Response):
    """Default response for OpenAPI."""

    def __init__(self):
        super().__init__(
            description="Successful response",
            content={
                "application/json": {
                    "schema": {
                        "type": "object",
                        "properties": {
                            "code": {"type": "integer", "example": 200},
                            "msg": {"type": "string", "example": "success"},
                            "data": {"type": "object", "example": {}},
                        },
                    }
                }
            },
        )


class JSONResponse(Response):
    """JSON response model for OpenAPI.

    >>> class ResponseModel(BaseModel):
    ...     code: int
    ...     msg: str
    ...     data: dict
    >>> j = JSONResponse(model=ResponseModel)
    >>> j.dict(exclude_none=True, exclude={"model"})
    """
    description: str = "Successful response"
    content: dict = Field(default_factory=dict)
    # 当 model 为一个字典时, 将其转换为 BaseModel, 则其中所有字段都是非必填的
    model: Union[Type[BaseModel], Type[Schema], dict]

    @root_validator(pre=True)
    def set_content(cls, values: dict) -> dict:
        """Set content field."""
        v = values["model"]
        if isinstance(v, dict):
            # 动态创建模型
            name = list(v.keys())[0]
            model = create_model(name, v[name])
            model.Config.schema_extra = {"example": v[name]}
            values["model"] = model
        elif issubclass(v, Schema):
            model = pydantic_from_marshmallow(schema=v)
            values["model"] = model
        elif issubclass(v, BaseModel):
            pass
        else:
            raise TypeError("model must be a dict or a pydantic model")
        return values

    @root_validator()
    def convert_content(cls, values: dict) -> dict:
        """Convert content field."""
        schema = values["model"]
        values["content"] = {
            "application/json": {
                "schema": {"$ref": f"#/components/schemas/{schema.__name__}"},
            }
        }
        return values
