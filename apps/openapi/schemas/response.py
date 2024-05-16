#  -*- coding: utf-8 -*-
#  Copyright (c) XYZ Robotics Inc. - All Rights Reserved
#  Unauthorized copying of this file, via any medium is strictly prohibited
#  Proprietary and confidential
#  Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022/11/15 下午1:38
from enum import IntEnum

from pydantic import BaseModel


class Code(IntEnum):
    success = 0
    error = -1


class STDResponseSchema(BaseModel):
    """Standard response schema for all APIs"""

    code: Code
    msg: str
    data: dict

    class Config:
        schema_extra = {"example": {"code": 0, "msg": "success", "data": {}}}


class DeprecatedSTDResponseSchema(STDResponseSchema):
    """Deprecated response schema for all APIs"""

    error: int = 0
    error_message: str = ""

    class Config:
        schema_extra = {
            "example": {
                "error": 0,
                "code": 0,
                "msg": "",
                "error_message": "",
                "data": {},
            }
        }
