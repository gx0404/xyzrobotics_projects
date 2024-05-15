# -*- coding: utf-8 -*-
"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022-07-26
"""
import uuid
from typing import List, Optional, Type, Union

from pydantic import BaseModel, Field, validator
from werkzeug.datastructures import FileStorage

from apps.exceptions import XYZValidationError
from apps.programs.dpt.views.design.models import Box as BoxDBModel
from apps.schemas import (
    SingleResponseSchema,
    PaginationResponseSchema,
    QuerySchema,
    MultiResponseSchema,
)
from apps.search_filter import FilterType, QueryFieldCollation, QueryFilter
from ..entity import BoxEntity


class BoxQuerySchema(QuerySchema):
    """用于纸箱数据的查询模式类."""

    @property
    def db_model(self) -> Type[BoxDBModel]:
        return BoxDBModel

    @property
    def default_sort(self) -> List[QueryFieldCollation]:
        """默认排序规则, 按主键id降序排列"""
        return [QueryFieldCollation(field="id", collation=-1)]

    @property
    def base_filters(self) -> List[FilterType]:
        """默认查询规则, 过滤已被逻辑删除的记录"""
        return [QueryFilter(op="eq", field="is_del", value=False)]


class BoxCreateSchema(BaseModel):
    """创建纸箱模式类."""

    name: str = Field(title="名称")
    length: float = Field(title="长(mm)", ge=10)
    width: float = Field(title="宽(mm)", ge=10)
    height: float = Field(title="高(mm)", ge=10)
    weight: float = Field(title="重量(kg)")
    scan_code: Optional[str] = Field(
        default_factory=lambda: uuid.uuid4().__str__(), title="条码"
    )
    img_file: Optional[FileStorage] = Field(default=None, title="图片文件对象")

    @validator("scan_code")
    def fill_scan_code(cls, v: Optional[str]) -> str:
        return uuid.uuid4().__str__() if v is None else v

    class Config:
        arbitrary_types_allowed = True


class BoxUpdateSchema(BoxCreateSchema):
    """更新纸箱模式类."""

    pass


class BoxBatchCreateSchema(BaseModel):
    """批量创建纸箱模式类."""

    __root__: List[BoxCreateSchema]


class BoxDeleteSchema(BaseModel):
    """记录的删除模式类."""

    ids: List[int] = Field(default_factory=list, title="待删除列表")

    class Config:
        schema_extra = {"example": {"ids": [1, 2, 3]}}

    @validator("ids", pre=True)
    def convert_ids(cls, v: Union[List[int], int]) -> List[int]:
        """转换ids为列表.

        如果ids为一个int类型，则转为list[int]

        Args:
            v: 一个或多个整数.

        Returns:
            List[int]: 待删除列表.
        """
        return [v] if isinstance(v, int) else v

    @validator("ids")
    def validate_ids(cls, v: List[int]) -> List[int]:
        """校验ids的长度.

        Args:
            v: 多个整数.

        Returns:
            List[int]: 待删除列表.
        """
        if not (0 < len(v) <= 100):
            raise XYZValidationError(
                error_message="ids length must be greater than 0 and less than 100."
            )
        return v


class BoxOutputSchema(BoxEntity):
    """输出纸箱数据模式类."""

    pass


class SingleBoxResponse(SingleResponseSchema[BoxOutputSchema]):
    """返回单个纸箱数据的响应类."""

    pass


class MultiBoxResponse(MultiResponseSchema[BoxOutputSchema]):
    """返回多个纸箱数据的响应模式."""

    pass


class PaginationBoxResponse(PaginationResponseSchema[BoxOutputSchema]):
    """返回多个纸箱数据的响应类."""

    pass
