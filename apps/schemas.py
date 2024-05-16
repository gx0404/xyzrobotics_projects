# -*- coding: utf-8 -*-
"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022-04-16
"""
import math
from datetime import datetime
from typing import Any, Generic, List, TypeVar, Union

from pydantic import BaseModel, Field, root_validator, PositiveInt, Extra
from pydantic.generics import GenericModel
from sqlalchemy import asc, desc

from apps.search_filter import FilterType, QueryFieldCollation

T = TypeVar("T", bound=BaseModel)


# 响应状态码
RESPONSE_STATUS_CODES = {
    -1: "error",
    0: "success",
}

SORTING_FUNCTION = {
    -1: desc,
    0: asc,
}


class QuerySchema(BaseModel):
    """用于查询的模式

    此类适用于搜索匹配的场景
    """

    page: PositiveInt = Field(default=1, description="当前页")
    page_size: PositiveInt = Field(default=10, description="每页数量")
    single: bool = Field(default=False, description="是否仅返回一条数据")
    filters: List[FilterType] = Field(default_factory=list, description="查询表达式列表")
    sort: List[QueryFieldCollation] = Field(default_factory=list, description="排序")

    class Config:
        orm_mode = True
        extra = Extra.forbid
        schema_extra = {
            "example": {
                "page": 1,
                "page_size": 20,
                "signle": False,
                "filters": [{"op": "eq", "field": "key", "value": "value"}],
                "sort": [{"field": "field", "collation": 1}]
            }
        }

    def to_sa_filters(self) -> List:
        """转换为sqlalchemy的数据库查询语句.

        Returns:
            返回一组用于SQLAlchemy的查询子句.
            e.g.
            filters = [
                Model.name.startswith("John"),
                Model.age > 18,
                Model.age < 25
            ]

            queryset = db.session.query(Model).filter(*filters)
        """
        if not self.filters:
            self.filters = self.base_filters
        else:
            self.filters += self.base_filters
        filters = [_filter.to_sa_filter(self.db_model) for _filter in self.filters]
        return filters

    def to_sa_collation(self) -> List:
        """转换排序规则

        input: [(field1, 0), (field2, -1)]

        output: [asc(Model.field1), desc(Model.field2)]

        query = db.session.query(Model).order_by(*[asc(Model.field1), desc(Model.field2)])
        """
        sort_list = []
        if not self.sort:
            self.sort = self.default_sort
        for _sort in self.sort:
            func = SORTING_FUNCTION.get(_sort.collation)
            try:
                # 等同于 asc(Model.field)
                ret = func(getattr(self.db_model, _sort.field))
            except AttributeError as err:
                # TODO(Yuhang Wu): 自定义异常, 表示该字段不能用于排序
                raise err

            sort_list.append(ret)
        return sort_list

    @property
    def db_model(self):
        """返回数据表模型"""
        raise NotImplementedError

    @property
    def default_sort(self) -> List[QueryFieldCollation]:
        """默认排序规则"""
        return []

    @property
    def base_filters(self) -> List[FilterType]:
        """基础过滤规则

        过滤规则 = 基础过滤规则 + 前端传入的过滤规则
        """
        return []


# 此处继承至GenericModel, 对于子类字段的约束才会生效.
class Response(GenericModel):
    """返回响应的固定模板.

    减少代码冗余, 响应格式固定化
    """

    code: int = Field(default=0, description="状态码")
    msg: str = Field(default=None, description="信息")

    class Config:
        json_encoders = {datetime: lambda x: x.strftime("%Y-%m-%d %H:%M:%S")}

    @root_validator()
    def fill_message(cls, values: dict) -> dict:
        """根据code值填充message.
        Args:
            values:
                {
                    "code": ?,
                    "message": None,
                }
        """
        code = values.get("code")
        values["msg"] = RESPONSE_STATUS_CODES.get(code, "Unknown Code")
        return values


class GetResponseSchema(Response, Generic[T]):
    """返回单条数据的响应模板.

    .. deprecated:: v1.4.0
        使用 ``SingleResponseSchema`` 代替.
    """

    data: Union[T, BaseModel, Any] = Field(description="单条数据")


SingleResponseSchema = GetResponseSchema


class MultiResponseSchema(Response, Generic[T]):
    """data 包含多条记录的响应模式."""

    data: List[Union[T, BaseModel, Any]] = Field(
        default_factory=list, description="多条数据"
    )


class ListResponseSchema(MultiResponseSchema, Generic[T]):
    """返回多条数据的响应模板.

    .. deprecated:: v1.4.0
        使用 ``PaginationResponseSchema`` 代替.
    """

    page: int = Field(description="当前页数")
    page_size: int = Field(description="页数量")
    total_page: int = Field(default=None, description="总页数")
    count: int = Field(description="总数量")

    @root_validator()
    def calculate_total_page(cls, values: dict) -> dict:
        """计算总页数

        根据每页数和总数计算得出分页分页数量

        Args:
            values:
                {
                    "page": 1,
                    "page_size": 20,
                    "total_page": None,
                    "count": 100,
                    "data": ...
                }

        Returns:
            总页数

        """
        count = values.get("count")
        page_size = values.get("page_size")
        values["total_page"] = math.ceil(count / page_size)
        return values


PaginationResponseSchema = ListResponseSchema
