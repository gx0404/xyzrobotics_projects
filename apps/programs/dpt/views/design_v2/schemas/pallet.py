# -*- coding: utf-8 -*-
"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022-07-26
"""
from typing import List, Optional, Type

from pydantic import BaseModel, Field, root_validator

from apps.programs.dpt.views.design.models import Pallet as PalletDBModel
from apps.schemas import GetResponseSchema, ListResponseSchema, QuerySchema
from apps.search_filter import FilterType, QueryFieldCollation, QueryFilter
from ..entity import PalletEntity


class PalletQuerySchema(QuerySchema):
    """用于托盘的查询模式类."""

    @property
    def db_model(self) -> Type[PalletDBModel]:
        return PalletDBModel

    @property
    def default_sort(self) -> List[QueryFieldCollation]:
        """默认排序规则, 按主键id降序排列"""
        return [QueryFieldCollation(field="id", collation=-1)]

    @property
    def base_filters(self) -> List[FilterType]:
        """默认查询规则, 过滤已被逻辑删除的记录"""
        return [QueryFilter(op="eq", field="is_del", value=False)]


class PalletCreateSchema(BaseModel):
    """创建托盘的模式类."""

    name: str = Field(title="托盘名称")
    length: float = Field(title="长(mm)", ge=10)
    width: float = Field(title="宽(mm)", ge=10)
    height: float = Field(title="高(mm)", ge=10)
    max_height: float = Field(title="最大高度(mm)", ge=10)
    max_weight: float = Field(title="最大承载重量(kg)", ge=10)
    # TODO(YuhangWu): 之后版本更改其类型, 不应该使用布尔类型
    pallet_type: bool  # True: 深框 False: 托盘
    thickness: Optional[float] = Field(
        default=None, title="壁厚", ge=0, description="只有深框,此字段才有效"
    )

    @root_validator()
    def root_validate(cls, values: dict) -> dict:
        # 如果托盘类型为托盘，即使指定了thickness，也给其覆盖为None.
        if not values["pallet_type"]:
            values["thickness"] = None
        return values

    class Config:
        schema_extra = {
            "example": {
                "name": "斗单切至方标世",
                "length": 47,
                "width": 15,
                "height": 66,
                "max_height": 77,
                "max_weight": 71,
                "pallet_type": False,
                "thickness": 51,
            }
        }


class PalletUpdateSchema(PalletCreateSchema):
    """托盘的更新模式类."""

    name: Optional[str] = None

    class Config:
        # name有唯一性, 不可更改.
        fields = {"name": {"exclude": True}}
        schema_extra = {
            "example": {
                "length": 47,
                "width": 15,
                "height": 66,
                "max_height": 77,
                "max_weight": 71,
                "pallet_type": False,
                "thickness": 51,
            }
        }


class PalletOutputSchema(PalletEntity):
    """托盘的输出模式类."""

    pass


class GetPalletResponse(GetResponseSchema[PalletOutputSchema]):
    """返回单个托盘记录的响应."""

    class Config:
        schema_extra = {
            "example": {
                "code": 0,
                "msg": "success",
                "data": {
                    "id": 16,
                    "name": "学气织",
                    "length": 76,
                    "width": 12,
                    "height": 76,
                    "max_height": 21,
                    "max_weight": 4,
                    "pallet_type": False,
                    "thickness": None,
                },
            }
        }


class ListPalletResponse(ListResponseSchema[PalletOutputSchema]):
    """返回多个托盘记录的响应."""

    class Config:
        schema_extra = {
            "example": {
                "code": 0,
                "msg": "success",
                "page": 1,
                "page_size": 4,
                "total_page": 1,
                "count": 2,
                "data": [
                    {
                        "id": 14,
                        "name": "学气争本织",
                        "length": 76,
                        "width": 12,
                        "height": 76,
                        "max_height": 21,
                        "max_weight": 4,
                        "pallet_type": False,
                        "thickness": None,
                    },
                    {
                        "id": 16,
                        "name": "学气织",
                        "length": 76,
                        "width": 12,
                        "height": 76,
                        "max_height": 21,
                        "max_weight": 4,
                        "pallet_type": False,
                        "thickness": None,
                    },
                ],
            }
        }
