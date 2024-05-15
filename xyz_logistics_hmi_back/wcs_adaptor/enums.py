# -*- coding: utf-8 -*-
"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022-05-05

定义的一些枚举类
"""
from enum import IntEnum


class TaskStatus(IntEnum):
    """ 任务状态枚举类，定义了任务的各种状态 """
    PENDING = UNSTARTED = 0  # 未开始
    STARTED = 1  # 已开始
    
    #拣配拣选完成
    DEPALFINISHED = 11
    #拣配180还原
    CHANGE_180 = 12
    
    #输送线空箱回收正常状态
    MULTI_NORMAL = 13
    #输送线空箱回收处理缓存区
    MULTI_CACHE = 14
            
    FINISHED = 2  # 已完成
    ERROR = 3  # 发生异常
    TERMINATED = 4  # 终止
    ENDED = 10  # 已结束


class TaskType(IntEnum):
    """ 任务类型枚举类，定义了任务的所属类型 """
    SINGLE_DEPAL = 0  # 单拆
    SINGLE_PAL = 1  # 单码
    MULTI_DEPAL = 2  # 混拆
    MULTI_PAL_ONLINE = 3  # 混码online
    MULTI_PAL_OFFLINE = 4  # 混码offline
    
    MERGE_TASK = 10 #合托

    PP_DEFAULT = 100    # Piece picking default type
    PP_CYCLE = 101      # 分拣站循环抓取模式，常用于压测


class OrderStatus(IntEnum):
    """ 订单状态枚举类，定义了订单的各种状态 """
    PENDING = 0  # 未开始
    STARTED = 1  # 已开始
    FINISHED = 2  # 已完成
    ERROR = 3  # 发生异常
    TERMINATED = 4  # 已终止
    ENDED = 10  # 已结束
