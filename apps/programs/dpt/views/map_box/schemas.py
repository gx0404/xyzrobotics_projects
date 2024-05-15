# -*- coding: utf-8 -*-
"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022-04-14
"""
from datetime import datetime
from decimal import Decimal
from typing import Type, List, Optional

from pydantic import BaseModel, Field, root_validator

from apps.programs.dpt.views.map_box.entity import MapBoxEntity
from apps.programs.dpt.views.map_box.models import MapBoxDBModel
from apps.schemas import SingleResponseSchema, PaginationResponseSchema
from apps.schemas import QuerySchema
from apps.search_filter import QueryFieldCollation, FilterType, QueryFilter


class MapBoxQuerySchema(QuerySchema):
    """用于纸箱查询的模式."""

    @property
    def db_model(self) -> Type[MapBoxDBModel]:
        return MapBoxDBModel

    @property
    def default_sort(self) -> List[QueryFieldCollation]:
        """默认排序规则, 按主键id降序排列"""
        return [QueryFieldCollation(field="id", collation=-1)]

    @property
    def base_filters(self) -> List[FilterType]:
        """默认查询规则, 过滤已被逻辑删除的记录"""
        return [QueryFilter(op="eq", field="is_del", value=False)]


class MapBoxUpdateSchema(BaseModel):
    """更新"""

    serial_number: str = Field(title="纸箱编号", description="纸箱编号")
    normal_length: Decimal = Field(gt=0, le=20000, title="标准长度", description="标准长度")
    normal_width: Decimal = Field(gt=0, le=20000, title="标准宽度", description="标准宽度")
    normal_height: Decimal = Field(gt=0, le=20000, title="标准高度", description="标准高度")
    normal_weight: Decimal = Field(gt=0, le=20000, title="标准重量", description="标准重量")
    real_length: Decimal = Field(gt=0, le=20000, title="实际长度", description="实际长度")
    real_width: Decimal = Field(gt=0, le=20000, title="实际宽度", description="实际宽度")
    real_height: Decimal = Field(gt=0, le=20000, title="实际高度", description="实际高度")
    real_weight: Decimal = Field(gt=0, le=20000, title="实际重量", description="实际重量")
    extra1: Optional[str] = Field(None, description="额外字段1")
    extra2: Optional[str] = Field(None, description="额外字段2")
    extra3: Optional[str] = Field(None, description="额外字段3")
    extra4: Optional[str] = Field(None, description="额外字段4")


class MapBoxCreateSchema(MapBoxUpdateSchema):
    """创建"""

    pass


class MapBoxDeleteSchema(BaseModel):
    """删除"""

    serial_number: str = Field(description="纸箱编号")


class MapBoxOuterSchema(MapBoxEntity):
    """向外部展示的模型"""

    pass


class SingleResponse(SingleResponseSchema[MapBoxOuterSchema]):
    """单条数据响应"""

    pass


class PaginationResponse(PaginationResponseSchema[MapBoxOuterSchema]):
    """分页数据响应"""

    extra_columns: List[dict] = Field(default_factory=list, description="额外的列")

    @root_validator()
    def root_validate(cls, values: dict) -> dict:
        """根据数据类型自动填充表头.

        extra_columns:
            [
                {
                    "column_field": "extra1",
                    "column_name": "yy",
                },
                ...
            ]
        """
        from apps.settings import settings

        extra_columns = []
        if settings.dpt_settings is None:
            raise ValueError("dpt_settings is None. Please check your settings.")
        for key, val in settings.dpt_settings.extra_columns_for_register_box.items():
            if val.enable:
                extra_columns.append({"key": key, "name": val.name})
        values["extra_columns"] = extra_columns
        return values
