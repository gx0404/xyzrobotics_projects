#  -*- coding: utf-8 -*-
#  Copyright (c) XYZ Robotics Inc. - All Rights Reserved
#  Unauthorized copying of this file, via any medium is strictly prohibited
#  Proprietary and confidential
#  Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022/9/20 下午6:03
from typing import Optional, Literal

from pydantic import Field, validator, root_validator

from apps import settings
from apps.settings import SkuLimit
from apps.ext.datastructs import BaseSKU
from wcs_adaptor.exceptions import ValidationError


class SkuInfo(BaseSKU):
    """物料信息.

    该类用于描述物料信息，包括物料编号、物料数量、物料重量、物料重量单位等。
    默认包含以下字段：
        * sku_id: 物料编号, 必填.
        * sku_type: 物料类型, 默认为 None.
        * weight_unit: 重量单位, 根据项目类型动态改变重量单位，拆码垛：kg，分拣站：g.
        * weight: 重量
        * width: 宽度
        * height: 高度
        * length: 长度

    使用示例::
        通过 `SkuInfo` 类创建物料信息::
        sku = SkuInfo(sku_id="sku_1", sku_type="type_1", weight=1.0, weight_unit="kg")
        sku.sku_id

        通过访问 `Task` 类的 `sku_info` 属性获取物料信息::
        task.sku_info

    如何修改字段类型以及默认值::
        默认情况下，长宽高重量等字段都是必填项，如果需要修改为可选项，可以通过 `Optional` 类型进行修改::
        class SkuInfo(BaseSKU):
            # 可以给一个默认值，比如 None, 0, 0.0 等
            height: Optional[float] = None

    如何校验(限制)长宽高等信息::
        1. 引入 Field 类, `from pydantic import Field`
        2. 更新 SkuInfo 中需要校验的字段，以下示例进仅供参考
            * `length` 大于等于 1 且小于等于 2, 即 1 <= length <= 2
              length: float = Field(ge=1, le=2)
            * `width` 大于 1 且小于 2, 即 1 < width < 2
              width: float = Field(gt=1, lt=2)
            * `weight` 小于 10, 即 weight < 10
              weight: float = Field(lt=10)
        `gt` 是 `greater than` 缩写，就是大于
        `lt` 是 `lesser than` 缩写，就是小于
        `ge` 是 `greater than or equal` 缩写，就是大于等于
        `ge` 是 `lesser than or equal` 缩写，就是小于等于

    如何将 `SkuInfo` 对象转为字典::
        sku = SkuInfo(sku_id="sku_1", sku_type=None, weight=1.0, weight_unit="kg", height=1.0, width=1.0, length=1.0)

        sku.dict()  # 返回所有字段构成的字典
        {'sku_id': 'sku_1', 'sku_type': None, 'weight': 1.0, 'weight_unit': 'kg', 'height': 1.0, 'width': 1.0, 'length': 1.0}

        sku.dict(exclude_none=True)  # 过滤掉空值
        {'sku_id': 'sku_1', 'weight': 1.0, 'weight_unit': 'kg', 'height': 1.0, 'width': 1.0, 'length': 1.0}

        sku.dict(exclude_none=True, exclude_defaults=True)  # 过滤掉空值和默认值
        {'sku_id': 'sku_1', 'weight': 1.0, 'weight_unit': 'kg'}

        sku.dict(exclude={"sku_id", "sku_type"})  # 过滤掉指定字段
        {'weight': 1.0, 'weight_unit': 'kg', 'height': 1.0, 'width': 1.0, 'length': 1.0}

    .. deprecated:: V1.1.0
        弃用 `sku_num` 字段，使用 `Task.target` 字段替代.

    .. versionadded:: V1.4.0
        新增 `weight_unit`，用于表示重量单位。
        根据项目类型动态改变重量单位，拆码垛：kg，分拣站：g
    """

    length: float
    width: float
    height: float
    weight: float
    weight_unit: Literal["kg", "g"] = Field(default="g", description="重量单位")
    sku_type: Optional[str] = None

    @validator("weight_unit")
    def validate_weight_unit(cls, v: Literal["kg", "g"]) -> Literal["kg", "g"]:
        """根据项目类型填充默认的重量单位.

        拆码垛: kg
        分拣站: g
        """
        if settings.PROJECT_TYPE == "dpt":
            return "kg"
        elif settings.PROJECT_TYPE == "pp":
            return "g"

    class Config:
        allow_population_by_field_name = True


class StrictSkuInfo(SkuInfo):
    """物料信息, 并且增加了长宽高重的校验限制

    该类用于描述物料信息，包括物料编号、物料数量、物料重量、物料重量单位等。
    默认包含以下字段：
        * sku_id: 物料编号, 必填.
        * sku_type: 物料类型, 默认为 None.
        * weight_unit: 重量单位, 根据项目类型动态改变重量单位，拆码垛：kg，分拣站：g.
        * weight: 重量
        * width: 宽度
        * height: 高度
        * length: 长度

    使用示例::
        通过 `SkuInfo` 类创建物料信息::
        sku = SkuInfo(sku_id="sku_1", sku_type="type_1", weight=1.0, weight_unit="kg")
        sku.sku_id

        通过访问 `Task` 类的 `sku_info` 属性获取物料信息::
        task.sku_info

    如何修改字段类型以及默认值::
        默认情况下，长宽高重量等字段都是必填项，如果需要修改为可选项，可以通过 `Optional` 类型进行修改::
        class StrictSkuInfo(SkuInfo):
            # 可以给一个默认值，比如 None, 0, 0.0 等
            height: Optional[float] = None

    如何校验(限制)长宽高等信息::
        1. 引入 Field 类, `from pydantic import Field`
        2. 更新 SkuInfo 中需要校验的字段，以下示例进仅供参考
            * `length` 大于等于 1 且小于等于 2, 即 1 <= length <= 2
              length: float = Field(ge=1, le=2)
            * `width` 大于 1 且小于 2, 即 1 < width < 2
              width: float = Field(gt=1, lt=2)
            * `weight` 小于 10, 即 weight < 10
              weight: float = Field(lt=10)
        `gt` 是 `greater than` 缩写，就是大于
        `lt` 是 `lesser than` 缩写，就是小于
        `ge` 是 `greater than or equal` 缩写，就是大于等于
        `ge` 是 `lesser than or equal` 缩写，就是小于等于

    如何将 `SkuInfo` 对象转为字典::
        sku = SkuInfo(sku_id="sku_1", sku_type=None, weight=1.0, weight_unit="kg", height=1.0, width=1.0, length=1.0)

        sku.dict()  # 返回所有字段构成的字典
        {'sku_id': 'sku_1', 'sku_type': None, 'weight': 1.0, 'weight_unit': 'kg', 'height': 1.0, 'width': 1.0, 'length': 1.0}

        sku.dict(exclude_none=True)  # 过滤掉空值
        {'sku_id': 'sku_1', 'weight': 1.0, 'weight_unit': 'kg', 'height': 1.0, 'width': 1.0, 'length': 1.0}

        sku.dict(exclude_none=True, exclude_defaults=True)  # 过滤掉空值和默认值
        {'sku_id': 'sku_1', 'weight': 1.0, 'weight_unit': 'kg'}

        sku.dict(exclude={"sku_id", "sku_type"})  # 过滤掉指定字段
        {'weight': 1.0, 'weight_unit': 'kg', 'height': 1.0, 'width': 1.0, 'length': 1.0}

    .. deprecated:: V1.1.0
        弃用 `sku_num` 字段，使用 `Task.target` 字段替代.

    .. versionadded:: V1.4.0
        新增 `weight_unit`，用于表示重量单位。
        根据项目类型动态改变重量单位，拆码垛：kg，分拣站：g
    """

    @root_validator()
    def validate_all(cls, values: dict) -> dict:
        sku_limit: SkuLimit = settings.common_settings.SKU_LIMIT
        
        if not sku_limit.min_length <= values["length"] <= sku_limit.max_length:
            raise ValidationError("SKU长度超出限制，当前长度{length}，范围限制[{min_length}, {max_length}]".format(
                length=values["length"],
                min_length=sku_limit.min_length,
                max_length=sku_limit.max_length
            ))
        
        if not sku_limit.min_width <= values["width"] <= sku_limit.max_width:
            raise ValidationError("SKU宽度超出限制，当前宽度{width}，范围限制[{min_width}, {max_width}]".format(
                width=values["width"],
                min_width=sku_limit.min_width,
                max_width=sku_limit.max_width
            ))
        
        if not sku_limit.min_height <= values["height"] <= sku_limit.max_height:
            raise ValidationError("SKU高度超出限制，当前高度{height}，范围限制[{min_height}, {max_height}]".format(
                height=values["height"],
                min_height=sku_limit.min_height,
                max_height=sku_limit.max_height
            ))
        
        if not sku_limit.min_weight <= values["weight"] <= sku_limit.max_weight:
            raise ValidationError("SKU重量超出限制，当前重量{weight}，范围限制[{min_weight}, {max_weight}]".format(
                weight=values["weight"],
                min_weight=sku_limit.min_weight,
                max_weight=sku_limit.max_weight
            ))

        return values