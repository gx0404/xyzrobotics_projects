# -*- coding: utf-8 -*-
"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022-08-01
"""
from datetime import datetime
from typing import List

from pydantic import Field

from apps.base_app.views.error_records.entity import ErrorRecordsEntity
from apps.models.error_records import ErrorRecordsDBModel
from apps.search_filter import QueryFieldCollation
from apps.schemas import QuerySchema, MultiResponseSchema, SingleResponseSchema


class ErrorRecordQuerySchema(QuerySchema):
    """用于查询的模式类"""
    @property
    def db_model(self):
        return ErrorRecordsDBModel

    @property
    def default_sort(self) -> List[QueryFieldCollation]:
        """默认排序规则"""
        return [QueryFieldCollation(field="id", collation=-1)]


class ErrorRecordsCreateSchema(ErrorRecordsEntity):
    """用于创建异常记录的模式类.

    用于创建时无需指定 id 和 task_id
    """
    id: None = None
    create_time: datetime = Field(
        default_factory=datetime.now,
        title="异常发生时间",
    )

    class Config:
        fields = {"id": {"exclude": True}, "task_id": {"exclude": True}}


class MultiErrorRecord(ErrorRecordsEntity):
    """用于列表页展示的记录.

    列表页展示，不需要展示 task, 所以可以屏蔽掉 task
    """
    class Config:
        fields = {"task": {"exclude": True}}


class SingleErrorRecordsResponse(SingleResponseSchema[ErrorRecordsEntity]):
    """单个详细的异常记录响应"""
    pass


class MultiErrorRecordsResponse(MultiResponseSchema[MultiErrorRecord]):
    """多个简略的异常记录响应"""
    pass
