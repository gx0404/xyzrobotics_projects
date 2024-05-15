#  -*- coding: utf-8 -*-
#  Copyright (c) XYZ Robotics Inc. - All Rights Reserved
#  Unauthorized copying of this file, via any medium is strictly prohibited
#  Proprietary and confidential
#  Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022/9/22 上午8:58
import warnings
from typing import Optional, Protocol

from pydantic import BaseModel
from pydantic import validator


class BaseSKU(BaseModel):
    """物料信息."""

    sku_id: str
    sku_num: Optional[int] = None

    @validator("sku_num")
    def validate_sku_num(cls, v: Optional[int]) -> Optional[int]:
        if isinstance(v, int):
            with warnings.catch_warnings():
                warnings.simplefilter("default", category=DeprecationWarning)
                warnings.warn(
                    "'SkuInfo.sku_num' 已弃用, 请使用 'Task.target_num' 代替.",
                    category=DeprecationWarning,
                )
            return v

    class Config:
        frozen = True  # 是否允许hash


class SKUProto(Protocol):
    """物料信息协议."""
    length: float
    width: float
    height: float
    weight: float
    sku_type: Optional[str] = None
