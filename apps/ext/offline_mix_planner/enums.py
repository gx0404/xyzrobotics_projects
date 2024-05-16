#  -*- coding: utf-8 -*-
#  Copyright (c) XYZ Robotics Inc. - All Rights Reserved
#  Unauthorized copying of this file, via any medium is strictly prohibited
#  Proprietary and confidential
#  Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022/10/21 下午4:09
import enum


class PlanningStatus(enum.Enum):
    # 等待中
    PENDING = 0
    # 规划中
    PLANNING = 1
    # 已完成
    FINISHED = 2
    # 规划失败
    FAIL = 3
