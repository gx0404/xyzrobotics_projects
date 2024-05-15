# -*- coding: utf-8 -*-
"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022-07-26
"""
import json
from typing import List, Literal, Optional, Type, Union

from pydantic import BaseModel, Field, root_validator, validator, Json

from apps.exceptions import XYZValidationError
from apps.programs.dpt.views.design.models import (
    Box as BoxDBModel,
    Design as PlanDBModel,
)
from apps.schemas import (
    QuerySchema,
    SingleResponseSchema,
    MultiResponseSchema,
    PaginationResponseSchema,
)
from apps.search_filter import (
    ConnectQueryFilter,
    FilterType,
    QueryFieldCollation,
    QueryFilter,
)
from .box import BoxCreateSchema
from ..entity import PlanEntity, Objects, BoxEntity, PalletEntity, Layout
from ..enums import BarcodeDirection, Flip, Mirror


class PlanQuerySchema(QuerySchema):
    """用于规划记录的搜索模式类."""

    @property
    def db_model(self) -> Type[PlanDBModel]:
        return PlanDBModel

    @property
    def default_sort(self) -> List[QueryFieldCollation]:
        """默认排序规则, 按主键id降序排列"""
        return [QueryFieldCollation(field="id", collation=-1)]

    @property
    def base_filters(self) -> List[FilterType]:
        """默认查询规则, 过滤已被逻辑删除的记录"""
        return [QueryFilter(op="eq", field="is_del", value=False)]

    def to_sa_sub_filters(self) -> List[FilterType]:
        """子查询"""
        sub_filters = []
        for _filter in self.filters:
            _filter: QueryFilter
            if isinstance(_filter, ConnectQueryFilter) or _filter.field.startswith(
                "p_"
            ):
                continue
            if _filter.field == "box_name":
                _filter.field = "name"
            # 用于子查询的过滤条件
            sub_filters.append(_filter.to_sa_filter(db_model=BoxDBModel))
        return sub_filters


class PlanDownloadSchema(PlanQuerySchema):
    """用于下载规划记录的模式."""

    ids: List[int] = Field(default_factory=list, title="主键值列表")
    filters: List[FilterType] = Field(default=[], description="查询表达式列表")
    sort: List[QueryFieldCollation] = Field(default=[], description="排序")

    class Config:
        fields = {
            "page": {"exclude": True},
            "page_size": {"exclude": True},
            "single": {"exclude": True},
        }

    @root_validator()
    def validate_ids(cls, values: dict) -> dict:
        """将ids中的内容，转为搜索条件.

        Examples:
            ids = [1, 2]

            convert to:

            [
                {
                    "in": [
                        {
                            "op": "in",
                            "field": "p_id",  # `p_`表示，该字段用于主表查询，即用于design表查询
                            "value": [1, 2]
                        }
                    ]
                }
            ]
        """
        # ids 优先
        if ids := values.get("ids"):
            values["filters"] = [QueryFilter(op="in", field="id", value=ids)]
        return values


class GeneratePlanInputSchema(BaseModel):
    """用于生成规划记录的输入模式类."""

    pallet_id: int
    box_id: int
    guillotine_packing: Literal[0, 1] = Field(description="0是非回型跺，1是回型跺")
    barcode_direction: BarcodeDirection
    mirror: Mirror
    flip: Flip
    layers: int = Field(title="层数", ge=0)

    @validator("guillotine_packing", pre=True)
    def validate_guillotine_packing(cls, v: Union[str, int]) -> int:
        return int(v) if isinstance(v, str) else v


class BatchGeneratePlanInputSchema(GeneratePlanInputSchema):
    """用于批量生成规划记录的输入模式类."""

    box_id: None = None
    boxes: List[BoxCreateSchema] = Field(default_factory=list, title="纸箱列表")

    class Config:
        fields = {"box_id": {"exclude": True}}


class PlanCreateSchema(GeneratePlanInputSchema):
    """规划记录的创建模式类."""

    layout: Union[Layout, List[List], str, None] = None
    objects: Objects
    image_url: Optional[str] = Field(title="图片链接")

    @root_validator(pre=True)
    def root_validate(cls, values: dict) -> dict:
        """校验数据."""
        layout = values.get("layout")
        objects: Objects = values.get("objects")
        if objects is None:
            if layout:
                objects = Objects.from_layout(layout)
                values["objects"] = objects
            else:
                raise XYZValidationError("'objects' 和 'layout' 不能同时为空.")
        else:
            values["objects"] = (
                Objects.parse_raw(objects)
                if isinstance(objects, (Json, dict, str))
                else objects
            )
        return values

    class Config:
        fields = {"layout": {"exclude": True}}
        use_enum_values = True


class PlanUpdateSchema(PlanCreateSchema):
    """垛型规划记录的更新模式类."""

    guillotine_packing: Optional[Literal[0, 1]] = None
    layers: Optional[int] = None
    barcode_direction: Optional[BarcodeDirection] = None
    flip: Optional[Flip] = None

    class Config:
        fields = {
            "guillotine_packing": {"exclude": True},
            "barcode_direction": {"exclude": True},
            "layers": {"exclude": True},
            "flip": {"exclude": True},
            "layout": {"exclude": True},
        }


class PlanPatchSchema(BaseModel):
    image_url: Optional[str] = None


class PlanDeleteSchema(BaseModel):
    """规划记录的删除模式类."""

    ids: List[int] = Field(default_factory=list, title="待删除列表")

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


class PlanSummaryOutputSchema(BaseModel):
    """简要数据输出.

    用于输出到列表页.
    """

    id: int
    box_id: int
    box_name: str
    box_length: float
    box_width: float
    box_height: float
    box_weight: float
    pallet_id: int
    pallet_name: str
    layers: int
    num_per_layer: int
    num_per_pallet: int
    image_url: Optional[str]

    @classmethod
    def from_entity(cls, entity: PlanEntity):
        return cls(
            id=entity.id,
            box_id=entity.box.id,
            box_name=entity.box.name,
            box_length=entity.box.length,
            box_width=entity.box.width,
            box_height=entity.box.height,
            box_weight=entity.box.weight,
            pallet_id=entity.pallet.id,
            pallet_name=entity.pallet.name,
            layers=entity.layers,
            num_per_layer=entity.num_per_layer,
            num_per_pallet=entity.num_per_pallet,
            image_url=entity.image_url,
        )


class PlanBaseOutputSchema(BaseModel):
    """用于输出规划记录的基础模式类."""

    box: BoxEntity
    pallet: PalletEntity
    guillotine_packing: Literal[0, 1]
    barcode_direction: BarcodeDirection
    mirror: Mirror
    flip: Flip
    layers: int = Field(ge=0, title="层数")
    num_per_layer: int = Field(default=0, ge=0, title="每层箱数")
    num_per_pallet: int = Field(default=0, ge=0, title="每托箱数")
    image_url: Optional[str] = Field(title="垛型图片链接")
    objects: Optional[Objects] = Field(default=None, title="物体奇偶层布局")


class PlanDetailOutputSchema(PlanBaseOutputSchema):
    """完整数据输出.

    .. versionchanged:: v1.4.0
        ``layout`` 隐藏输出，由 ``objects`` 代替
    """

    id: int
    last_id: Optional[int] = Field(gt=1, title="上一条记录的主键值.")
    next_id: Optional[int] = Field(gt=1, title="下一条记录的主键值.")


class SinglePlanResponse(SingleResponseSchema[PlanDetailOutputSchema]):
    """返回一个垛型规划记录的响应."""

    class Config:
        schema_extra = {
            "example": {
                "code": 0,
                "msg": "success",
                "data": {
                    "id": 57,
                    "box": {
                        "id": 126,
                        "name": "d737ed18-be6d-4aff-8826-c7cad27e9ee2",
                        "length": 1000,
                        "width": 1000,
                        "height": 500,
                        "weight": 15,
                        "scan_code": "d737ed18-be6d-4aff-8826-c7cad27e9ee2",
                        "img_url": None,
                    },
                    "pallet": {
                        "id": 111,
                        "name": "a0c1bdb3-fe0b-49c8-833c-ab659abfb9ab",
                        "length": 2000,
                        "width": 1500,
                        "height": 2000,
                        "max_height": 3000,
                        "max_weight": 10000,
                        "pallet_type": False,
                        "thickness": None,
                    },
                    "guillotine_packing": 0,
                    "barcode_direction": 0,
                    "mirror": 0,
                    "flip": "x",
                    "layers": 0,
                    "num_per_layer": 2,
                    "num_per_pallet": 0,
                    "image_url": None,
                    "objects": {
                        "odd_layer": [
                            {
                                "label": 0,
                                "x": 499.99997999999994,
                                "y": 0.000015000000075815478,
                                "rotation": 1,
                                "place_drop_buffer": 0,
                            },
                            {
                                "label": 1,
                                "x": -500.00002000000006,
                                "y": 0.000015000000075815478,
                                "rotation": 1,
                                "place_drop_buffer": 0,
                            },
                        ],
                        "even_layer": [
                            {
                                "label": 0,
                                "x": 499.99997999999994,
                                "y": 0.000015000000075815478,
                                "rotation": 1,
                                "place_drop_buffer": 0,
                            },
                            {
                                "label": 1,
                                "x": -500.00002000000006,
                                "y": 0.000015000000075815478,
                                "rotation": 1,
                                "place_drop_buffer": 0,
                            },
                        ],
                    },
                    "last_id": None,
                    "next_id": 56,
                },
            }
        }


class MultiPlanResponse(MultiResponseSchema[PlanDetailOutputSchema]):
    """返回多个垛型规划记录的响应."""

    pass


class PlanPaginationResponse(PaginationResponseSchema[PlanSummaryOutputSchema]):
    """分页展示垛型规划记录的响应."""

    class Config:
        schema_extra = {
            "example": {
                "code": 0,
                "msg": "success",
                "page": 1,
                "page_size": 10,
                "total_page": 1,
                "count": 8,
                "data": [
                    {
                        "id": 118,
                        "box_id": 139,
                        "box_name": "e018f89b-790e-4cff-b81f-4997379aae3a",
                        "box_length": 1000,
                        "box_width": 1000,
                        "box_height": 1000,
                        "box_weight": 20,
                        "pallet_id": 120,
                        "pallet_name": "4069a5e2-a6d4-45fd-9856-875d4e0c2dc7",
                        "layers": 2,
                        "num_per_layer": 2,
                        "num_per_pallet": 4,
                        "image_url": "/static/images/pallet_design/1659084200.png",
                    }
                ],
            }
        }


class OneGenerateResultResponse(SingleResponseSchema[PlanDetailOutputSchema]):
    """返回生成一个垛型规划记录的结果."""

    pass


class MultiGenerateResultsResponse(MultiResponseSchema[PlanDetailOutputSchema]):
    """返回多个详细垛型规划记录的响应."""

    pass
