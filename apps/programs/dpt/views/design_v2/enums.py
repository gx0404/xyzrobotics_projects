# -*- coding: utf-8 -*-
"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022-07-26
"""
from enum import Enum, IntEnum


class TranslateMixin:
    # 用于sqlalchemy转译时使用
    def translate(self, _escape_table):
        return self.value  # type: ignore


# 条码方向
class BarcodeDirection(TranslateMixin, IntEnum):
    DEFAULT = 0  # 无条码
    TWO_LONG_SIDE = 1  # 两个长边
    ONE_LONG_SIDE = 2  # 一个长边
    TWO_SHORT_SIDE = 3  # 两个短边
    ONE_SHORT_SIDE = 4  # 一个长边


# 奇偶层变化方式
class Mirror(TranslateMixin, IntEnum):
    DEFAULT = 0  # 不变化
    HORIZONTAL_SYMMETRY = 1  # 水平对称
    VERTICAL_SYMMETRY = 2  # 垂直对称
    DIAGONAL_SYMMETRY = 3  # 对角线对称（旋转180度）
    COUNTERCLOCKWISE_ROTATION_90 = 4  # 逆时针旋转90度


# 翻转方式
class Flip(TranslateMixin, Enum):
    DEFAULT = ""
    X = "x"  # 按x轴翻转
    Y = "y"  # 按y轴翻转


# 逆时针旋转方式
class ClockwiseRotation(TranslateMixin, IntEnum):
    _0 = 1
    _90 = 2
    _180 = 3
    _270 = 4
