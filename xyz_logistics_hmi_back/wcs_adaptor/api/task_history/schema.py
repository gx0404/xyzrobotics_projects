# -*- coding: utf-8 -*-
"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022-07-04
"""
from typing import List

from pydantic import Field, root_validator

from apps.schemas import (
    GetResponseSchema,
    ListResponseSchema,
    QuerySchema,
    RESPONSE_STATUS_CODES
)
from apps.search_filter import QueryFieldCollation
from wcs_adaptor.models import HistoryTaskModel as TaskHistoryDBModel
from .entity import TaskHistory as TaskHistoryEntity


class TaskHistoryQuerySchema(QuerySchema):

    @property
    def db_model(self):
        return TaskHistoryDBModel

    @property
    def default_sort(self) -> List[QueryFieldCollation]:
        return [QueryFieldCollation(field="id", collation=-1)]


class TaskHistoryBaseSchema(TaskHistoryEntity):
    """用于展示粗略的历史任务信息.

    常用于历史任务列表的展示.
    """

    class Config:
        fields = {
            "customized_data": {"exclude": True},
            "error_code": {"exclude": True},
            "error_msg": {"exclude": True},
            "from_ws": {"exclude": True},
            "to_ws": {"exclude": True}
        }


class TaskHistorySchema(TaskHistoryEntity):
    """用于展示详细的历史任务."""
    pass


# TODO(YuhangWu): 原为message，修改为msg字段
#   由于基类GetResponseSchema采用了message，所以在此处复写。
#   将在1.3.0版本修改此基类的message字段为msg，到时可以删除以下的复写内容。
class Response(GetResponseSchema[TaskHistorySchema]):
    msg: str = Field(default=None, description="信息")

    class Config:
        fields = {
            "message": {
                "exclude": True
            }
        }

    @root_validator()
    def fill_message(cls, values: dict) -> dict:
        """根据code值填充message.
        Args:
            values:
                {
                    "code": ?,
                    "msg": None,
                }
        """
        code = values.get("code")
        values["msg"] = RESPONSE_STATUS_CODES.get(code, "Unknown Code")
        return values


# TODO(YuhangWu): 同上方的Response.
class ListResponse(ListResponseSchema[TaskHistoryBaseSchema]):
    msg: str = Field(default=None, description="信息")

    class Config:
        fields = {
            "message": {
                "exclude": True
            }
        }

    @root_validator()
    def fill_message(cls, values: dict) -> dict:
        """根据code值填充message.
        Args:
            values:
                {
                    "code": ?,
                    "msg": None,
                }
        """
        code = values.get("code")
        values["msg"] = RESPONSE_STATUS_CODES.get(code, "Unknown Code")
        return values
