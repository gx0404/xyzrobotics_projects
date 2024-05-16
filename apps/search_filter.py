# -*- coding: utf-8 -*-
"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022-04-16
"""
from typing import Any, Callable, List, Type, TypeVar, Union

from flask_sqlalchemy import Model
from pydantic import BaseModel, Field
from sqlalchemy.sql import and_, not_, or_
from sqlalchemy.sql.operators import ColumnOperators

from apps.exceptions import XYZBaseError, XYZUnknownSearchOperator


def is_null(op: ColumnOperators, *args):
    """判断当前字段是否为空
    
    Args:
        op: 列操作符对象

    Returns:

    """
    return op.is_(None)


def is_not_null(op: ColumnOperators, *args):
    """判断当前字段是否不为空
    
    Args:
        op: 列操作符对象

    Returns:

    """
    return op.isnot(None)


def not_contains(op: ColumnOperators, val: str):
    """不包含某个子字符串

    Args:
        op: 列操作对象
        val: 结尾值
    """
    return not_(op.contains(val))


# 所有可用的操作符
operators = {
    "and": "__and__",
    "or": "__or__",
    "eq": "__eq__",
    "neq": "__ne__",
    "gt": "__gt__",
    "lt": "__lt__",
    "ge": "__ge__",
    "le": "__le__",
    "in": "in_",
    "not_in": "notin_",
    "is_null": is_null,
    "is_not_null": is_not_null,
    "startswith": "startswith",
    "endswith": "endswith",
    "contains": "contains",
    "not_contains": not_contains,
}


class BaseQueryFilter(BaseModel):

    class Config:
        orm_mode = True

    def to_sa_filter(self, db_model: Model):
        """
        
        Args:
            db_model: SQLAlchemy数据模型对象

        Returns:
            返回过滤规则
            
        """
        pass


class QueryFilter(BaseQueryFilter):
    op: str = Field(description="表达式")
    field: str = Field(description="字段名")
    value: Any = Field(description="值")

    def get_func_by_op(self) -> Union[Callable, str]:
        """根据操作符获取对应的处理函数.

        Returns:
            Callable: 可调用对象，或者是函数名.
        """
        try:
            return operators[self.op]
        except KeyError:
            # 未知错误操作符
            raise XYZUnknownSearchOperator(f"{self.op}, 未知操作符") from None

    def to_sa_filter(self, db_model: Type[Model]):
        """根据数据库模型，转为用于sqlalchemy查询的过滤条件.

        模型.字段 操作符 值
        getattr(cls, field) op value
        例如:
            MapBoxModel.serial_number == 1
            MapBoxModel.length < 1.0
            MapBoxModel.name like "aa%"

        Args:
            db_model: SQLAlchemy数据模型对象

        Returns:

        """
        column = self.get_column(db_model)
        handle_function = self.get_func_by_op()
        return self.dispatch_function(column, handle_function)

    def dispatch_function(
        self,
        column: ColumnOperators,
        func: Union[Callable, str]
    ):
        """调用函数."""
        if isinstance(func, str):
            # ModelBoxModel.serial_number.__eq__(self.val)
            return getattr(column, func)(self.value)
        else:
            # e.g. ModelBoxModel.serial_number.is_(None)
            return func(column, self.value)

    def get_column(self, db_model: Type[Model]) -> ColumnOperators:
        """获取列对象."""
        try:
            column = getattr(db_model, self.field)  # DBModel.id
        except AttributeError as err:
            raise XYZBaseError(error_message=f"{err}") from err
        return column


class ConnectQueryFilter(BaseQueryFilter):
    """连接符"""
    pass


class QueryAndFilter(ConnectQueryFilter):
    and_: List["FilterType"] = Field(alias="and")

    def to_sa_filter(self, db_model: Model):
        filters = []
        for _filter in self.and_:
            filters.append(_filter.to_sa_filter(db_model))
        return and_(*filters)


class QueryOrFilter(ConnectQueryFilter):
    or_: List["FilterType"] = Field(alias="or")

    def to_sa_filter(self, db_model: Model):
        filters = []
        for _filter in self.or_:
            filters.append(_filter.to_sa_filter(db_model))
        return or_(*filters)


class QueryFieldCollation(BaseModel):
    field: str
    collation: int = Field(default=0, description="排序规则, 0: 升序, -1: 降序")


FilterType = TypeVar("FilterType", QueryFilter, QueryAndFilter, QueryOrFilter)
QueryOrFilter.update_forward_refs()
QueryAndFilter.update_forward_refs()
